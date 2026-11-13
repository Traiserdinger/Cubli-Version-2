/**
 * @file    app_main.c
 * @brief   Điều phối toàn hệ thống (không gọi HAL trực tiếp, chỉ qua bsp_port.h).
 *
 * LỊCH CHẠY (mỗi mẫu IMU = 1 ms, nhịp do chân INT của IMU1 tạo ra):
 *
 *   Mỗi mẫu      : đọc 2 IMU qua SPI -> Align_IMU_Axes -> trung bình
 *                  -> EKF_Predict + EKF_AccumAccel -> tính Euler -> nạp frame.
 *   Việc "chậm" được chia đều vào 5 khe, mỗi mẫu chạy 1 khe (=> mỗi việc 200 Hz),
 *   để không mẫu nào phải gánh quá nhiều tính toán (STM32F103 không có FPU):
 *     khe 0 : EKF_CovPropagate  (P = F P F' + Q)
 *     khe 1 : EKF_UpdateAccel   (hiệu chỉnh Roll/Pitch)
 *     khe 2 : đọc từ kế (I2C)
 *     khe 3 : EKF_UpdateMag     (hiệu chỉnh Yaw, khi có mẫu từ kế mới ~100 Hz)
 *     khe 4 : đọc INA219 (I2C, luân phiên điện áp / dòng), kiểm tra sức khỏe,
 *             dò lại cảm biến mất kết nối, MAGCAL
 *
 *   Ước lượng thời gian tính toán trên STM32F103 @72 MHz (đếm lệnh bằng bộ giả
 *   lập Cortex-M3, chưa kể thời gian trên dây SPI/I2C): nhịp nhanh ~235 µs/mẫu,
 *   khe chậm nặng nhất ~200 µs. Kiểm tra thực tế bằng g_app_diag.step_us_max
 *   (phải < 1000) và g_app_diag.drdy_missed (phải đứng yên).
 */
#include "app_main.h"
#include "app_config.h"
#include "bsp_port.h"
#include "sensors_driver.h"
#include "ekf_fusion.h"
#include "comm_protocol.h"
#include "system_monitor.h"

#include <string.h>

/* -------------------------------------------------------------------------- */
/* Hằng số                                                                    */
/* -------------------------------------------------------------------------- */

#define APP_SAMPLE_PERIOD_US     (1000000UL / CFG_IMU_ODR_HZ)
#define APP_SLOT_COUNT           5U
#define APP_DRDY_TIMEOUT_MS      20U      /* không thấy xung INT lâu hơn -> lấy mẫu theo đồng hồ */
#define APP_MAG_TIMEOUT_MS       500U     /* từ kế không ra mẫu lâu hơn -> coi như lỗi */
#define APP_I2C_FAIL_LIMIT       5U       /* số lần lỗi I2C liên tiếp -> coi như mất thiết bị */
#define APP_REPROBE_PERIOD       200U     /* số lần chạy khe 4 giữa hai lần dò lại (~1 s) */
#define APP_INIT_MAX_WINDOWS     5U       /* chờ mạch đứng yên tối đa bấy nhiêu cửa sổ rồi vẫn khởi tạo */
#define APP_STILL_GYRO_STD       (0.02f)  /* rad/s: dao động gyro nhỏ hơn mức này = đứng yên */
#define APP_STILL_GYRO_MEAN      (0.2f)   /* rad/s: bias hợp lý của MPU6500 nhỏ hơn mức này */

/* -------------------------------------------------------------------------- */
/* Biến                                                                       */
/* -------------------------------------------------------------------------- */

app_diag_t g_app_diag;

static const sensors_bus_t s_bus = {
    BSP_SpiTransfer, BSP_SpiSetSpeed, BSP_I2cWrite, BSP_I2cRead, BSP_DelayMs
};

static mpu6500_t s_imu1;
static mpu6500_t s_imu2;
static mag_t     s_mag;
static ina219_t  s_ina;
static ekf_t     s_ekf;

/* Nhịp lấy mẫu */
static uint32_t s_last_ts_us;
static uint32_t s_last_drdy_ms;
static bool     s_drdy_alive;
static bool     s_first_sample;
static uint8_t  s_slot;

/* Khởi tạo EKF: cộng dồn mẫu lúc đứng yên */
static struct {
    vec3f_t  acc_sum;
    vec3f_t  gyro_sum;
    vec3f_t  gyro_sq;
    vec3f_t  mag_sum;
    uint32_t n;
    uint32_t mag_n;
    uint32_t windows;
} s_init;

/* Từ kế */
static vec3f_t  s_mag_body;
static bool     s_mag_fresh;
static bool     s_mag_drdy_alive;
static uint32_t s_mag_last_ok_ms;
static uint32_t s_mag_last_drdy_ms;
static uint8_t  s_mag_fail;

/* INA219 */
static uint8_t  s_ina_fail;
static bool     s_ina_turn_shunt;

/* Sức khỏe / dò lại */
static uint32_t s_reprobe_div;
static uint8_t  s_reprobe_turn;

/* MAGCAL */
static struct {
    bool     active;
    uint32_t start_ms;
    uint32_t n;
    vec3f_t  min;
    vec3f_t  max;
} s_magcal;

/* -------------------------------------------------------------------------- */
/* Cầu nối BSP <-> các module (đảo chiều phụ thuộc)                           */
/* -------------------------------------------------------------------------- */

static void app_led_write(sys_led_t led, bool on)
{
    BSP_LedWrite((uint8_t)((led == SYS_LED_HEARTBEAT) ? BSP_LED_HEARTBEAT : BSP_LED_FAULT), on);
}

/* Chạy trong ngắt USART1/DMA. */
static void app_uart_error_isr(uint32_t bsp_flags, bool tx_aborted)
{
    uint32_t f = 0UL;
    if ((bsp_flags & BSP_UART_ERR_OVERRUN) != 0UL) { f |= COMM_ERR_UART_OVERRUN; }
    if ((bsp_flags & BSP_UART_ERR_DMA) != 0UL)     { f |= COMM_ERR_DMA; }
    Comm_OnUartError(f, tx_aborted);
}

/* -------------------------------------------------------------------------- */
/* Tiện ích                                                                   */
/* -------------------------------------------------------------------------- */

static int16_t clamp_i16(int32_t v)
{
    if (v > 32767)  { return 32767; }
    if (v < -32768) { return -32768; }
    return (int16_t)v;
}

static uint16_t clamp_u16(int32_t v)
{
    if (v > 65535) { return 65535U; }
    if (v < 0)     { return 0U; }
    return (uint16_t)v;
}

/** Độ -> 0.01 độ, làm tròn về số gần nhất. */
static int16_t deg_to_cdeg(float deg)
{
    float v = deg * 100.0f;
    return clamp_i16((int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f)));
}

/** Viết số thực dạng [-]I.FFFF vào chuỗi (không dùng printf để tiết kiệm Flash). */
static char *put_fixed4(char *p, float v)
{
    if (v < 0.0f) {
        *p++ = '-';
        v = -v;
    }
    if (v > 999.0f) {
        v = 999.0f;
    }
    uint32_t scaled = (uint32_t)(v * 10000.0f + 0.5f);
    uint32_t ip = scaled / 10000UL;
    uint32_t fp = scaled % 10000UL;

    char tmp[4];
    int n = 0;
    do {
        tmp[n++] = (char)('0' + (ip % 10UL));
        ip /= 10UL;
    } while ((ip > 0UL) && (n < 4));
    while (n > 0) {
        *p++ = tmp[--n];
    }
    *p++ = '.';
    *p++ = (char)('0' + (fp / 1000UL));
    *p++ = (char)('0' + ((fp / 100UL) % 10UL));
    *p++ = (char)('0' + ((fp / 10UL) % 10UL));
    *p++ = (char)('0' + (fp % 10UL));
    return p;
}

static char *put_str(char *p, const char *s)
{
    while (*s != '\0') {
        *p++ = *s++;
    }
    return p;
}

/* -------------------------------------------------------------------------- */
/* Khởi tạo EKF từ dữ liệu đứng yên                                           */
/* -------------------------------------------------------------------------- */

static void app_init_window_reset(void)
{
    memset(&s_init.acc_sum, 0, sizeof(s_init.acc_sum));
    memset(&s_init.gyro_sum, 0, sizeof(s_init.gyro_sum));
    memset(&s_init.gyro_sq, 0, sizeof(s_init.gyro_sq));
    memset(&s_init.mag_sum, 0, sizeof(s_init.mag_sum));
    s_init.n = 0U;
    s_init.mag_n = 0U;
}

static void app_ekf_restart(void)
{
    EKF_Reset(&s_ekf, NULL);
    app_init_window_reset();
    s_init.windows = 0U;
}

static void app_ekf_init_step(const imu_sample_t *imu)
{
    s_init.acc_sum.x  += imu->accel_g.x;   s_init.acc_sum.y  += imu->accel_g.y;   s_init.acc_sum.z  += imu->accel_g.z;
    s_init.gyro_sum.x += imu->gyro_rads.x; s_init.gyro_sum.y += imu->gyro_rads.y; s_init.gyro_sum.z += imu->gyro_rads.z;
    s_init.gyro_sq.x  += imu->gyro_rads.x * imu->gyro_rads.x;
    s_init.gyro_sq.y  += imu->gyro_rads.y * imu->gyro_rads.y;
    s_init.gyro_sq.z  += imu->gyro_rads.z * imu->gyro_rads.z;
    s_init.n++;
    if (s_init.n < (uint32_t)CFG_EKF_INIT_SAMPLES) {
        return;
    }

    float inv = 1.0f / (float)s_init.n;
    vec3f_t acc  = { s_init.acc_sum.x * inv,  s_init.acc_sum.y * inv,  s_init.acc_sum.z * inv };
    vec3f_t gyro = { s_init.gyro_sum.x * inv, s_init.gyro_sum.y * inv, s_init.gyro_sum.z * inv };
    /* Phương sai gyro trong cửa sổ = E[x^2] - (E[x])^2. */
    float vx = s_init.gyro_sq.x * inv - gyro.x * gyro.x;
    float vy = s_init.gyro_sq.y * inv - gyro.y * gyro.y;
    float vz = s_init.gyro_sq.z * inv - gyro.z * gyro.z;
    const float var_lim  = APP_STILL_GYRO_STD * APP_STILL_GYRO_STD;
    const float mean_lim = APP_STILL_GYRO_MEAN;

    bool still = (vx < var_lim) && (vy < var_lim) && (vz < var_lim) &&
                 (gyro.x < mean_lim) && (gyro.x > -mean_lim) &&
                 (gyro.y < mean_lim) && (gyro.y > -mean_lim) &&
                 (gyro.z < mean_lim) && (gyro.z > -mean_lim);

    vec3f_t mag = { 0.0f, 0.0f, 0.0f };
    bool have_mag = (s_init.mag_n >= 3U);
    if (have_mag) {
        float minv = 1.0f / (float)s_init.mag_n;
        mag.x = s_init.mag_sum.x * minv; mag.y = s_init.mag_sum.y * minv; mag.z = s_init.mag_sum.z * minv;
    }

    s_init.windows++;
    if (still || (s_init.windows >= APP_INIT_MAX_WINDOWS)) {
        /* Đứng yên: trung bình gyro chính là bias. Không đứng yên: để EKF tự tìm bias. */
        (void)EKF_InitAttitude(&s_ekf, &acc, have_mag ? &mag : NULL, still ? &gyro : NULL);
        s_init.windows = 0U;
    }
    app_init_window_reset();
}

/* -------------------------------------------------------------------------- */
/* Các khe "chậm"                                                             */
/* -------------------------------------------------------------------------- */

static void app_magcal_feed(const vec3f_t *raw)
{
    if (!s_magcal.active) {
        return;
    }
    if (s_magcal.n == 0U) {
        s_magcal.min = *raw;
        s_magcal.max = *raw;
    } else {
        if (raw->x < s_magcal.min.x) { s_magcal.min.x = raw->x; }
        if (raw->y < s_magcal.min.y) { s_magcal.min.y = raw->y; }
        if (raw->z < s_magcal.min.z) { s_magcal.min.z = raw->z; }
        if (raw->x > s_magcal.max.x) { s_magcal.max.x = raw->x; }
        if (raw->y > s_magcal.max.y) { s_magcal.max.y = raw->y; }
        if (raw->z > s_magcal.max.z) { s_magcal.max.z = raw->z; }
    }
    s_magcal.n++;
}

static void app_magcal_finish(void)
{
    char line[96];
    char *p = line;

    s_magcal.active = false;

    /* Hard-iron: tâm của khoảng [min, max] từng trục.
     * Soft-iron (đơn giản): kéo bán kính 3 trục về bằng bán kính trung bình. */
    float rx = 0.5f * (s_magcal.max.x - s_magcal.min.x);
    float ry = 0.5f * (s_magcal.max.y - s_magcal.min.y);
    float rz = 0.5f * (s_magcal.max.z - s_magcal.min.z);

    /* Mỗi trục phải quét được ít nhất ±0.1 gauss, nếu không là chưa xoay đủ hướng. */
    if ((s_magcal.n < 100U) || (rx < 0.1f) || (ry < 0.1f) || (rz < 0.1f)) {
        p = put_str(p, "MAGCAL FAIL (xoay chua du huong)\r\n");
        *p = '\0';
        (void)Comm_SendText(line);
        return;
    }
    float ravg = (rx + ry + rz) / 3.0f;

    s_mag.offset.x = 0.5f * (s_magcal.max.x + s_magcal.min.x);
    s_mag.offset.y = 0.5f * (s_magcal.max.y + s_magcal.min.y);
    s_mag.offset.z = 0.5f * (s_magcal.max.z + s_magcal.min.z);
    s_mag.scale.x  = ravg / rx;
    s_mag.scale.y  = ravg / ry;
    s_mag.scale.z  = ravg / rz;

    /* Độ lớn từ trường tham chiếu của EKF phải học lại theo hiệu chuẩn mới. */
    s_ekf.mag_ref_norm = 0.0f;

    p = put_str(p, "MAGCAL OK OFFSET ");
    p = put_fixed4(p, s_mag.offset.x); *p++ = ' ';
    p = put_fixed4(p, s_mag.offset.y); *p++ = ' ';
    p = put_fixed4(p, s_mag.offset.z);
    p = put_str(p, " SCALE ");
    p = put_fixed4(p, s_mag.scale.x); *p++ = ' ';
    p = put_fixed4(p, s_mag.scale.y); *p++ = ' ';
    p = put_fixed4(p, s_mag.scale.z);
    p = put_str(p, "\r\n");
    *p = '\0';
    (void)Comm_SendText(line);
}

/* Lệnh không thuộc giao thức truyền (gọi từ Comm_Process, ngữ cảnh vòng lặp chính). */
static void app_on_user_cmd(const char *line)
{
    if (strcmp(line, "MAGCAL") == 0) {
        s_magcal.active   = true;
        s_magcal.start_ms = BSP_Millis();
        s_magcal.n        = 0U;
    }
}

static void app_mag_step(uint32_t now_ms)
{
    if (!s_mag.present) {
        return;
    }

    bool hint = BSP_MagDrdyTake();
    if (hint) {
        s_mag_last_drdy_ms = now_ms;
        s_mag_drdy_alive = true;
    } else if (s_mag_drdy_alive && ((uint32_t)(now_ms - s_mag_last_drdy_ms) > 100U)) {
        s_mag_drdy_alive = false;        /* chân DATA_RD im lặng: chuyển sang hỏi trạng thái */
    } else {
        /* giữ nguyên */
    }
    if (!hint && s_mag_drdy_alive) {
        return;                          /* chưa có mẫu mới, khỏi tốn một giao dịch I2C */
    }

    vec3f_t raw;
    vec3f_t body;
    mag_read_t r = Mag_Read(&s_mag, &s_bus, hint, &raw, &body);
    if (r == MAG_READ_OK) {
        s_mag_fail = 0U;
        s_mag_last_ok_ms = now_ms;
        s_mag_body = body;
        s_mag_fresh = true;
        g_app_diag.mag_samples++;
        g_app_diag.mag_gauss[0] = body.x; g_app_diag.mag_gauss[1] = body.y; g_app_diag.mag_gauss[2] = body.z;
        app_magcal_feed(&raw);
        if (!s_ekf.initialized) {
            s_init.mag_sum.x += body.x; s_init.mag_sum.y += body.y; s_init.mag_sum.z += body.z;
            s_init.mag_n++;
        }
    } else if (r == MAG_READ_BUS_ERROR) {
        if (++s_mag_fail >= APP_I2C_FAIL_LIMIT) {
            s_mag.present = false;
        }
    } else {
        /* chưa có mẫu / tràn thang: bỏ qua */
    }
}

static void app_ina_step(void)
{
    if (!s_ina.present) {
        return;
    }
    bool ok = s_ina_turn_shunt ? INA219_ReadShunt(&s_ina, &s_bus) : INA219_ReadBus(&s_ina, &s_bus);
    s_ina_turn_shunt = !s_ina_turn_shunt;
    if (ok) {
        s_ina_fail = 0U;
        g_app_diag.ina_samples++;
        g_app_diag.bus_mv = s_ina.bus_mv;
        g_app_diag.current_ma = s_ina.current_ma;
    } else if (++s_ina_fail >= APP_I2C_FAIL_LIMIT) {
        s_ina.present = false;
    } else {
        /* lỗi lẻ: thử lại lần sau */
    }
}

static void app_health_step(uint32_t now_ms)
{
    /* --- EKF --- */
    if (!EKF_IsHealthy(&s_ekf)) {
        SysMon_RaiseTimed(SYS_FAULT_EKF, (uint32_t)CFG_FAULT_EVENT_HOLD_MS);
        g_app_diag.ekf_resets++;
        app_ekf_restart();
    }
    if (!s_imu1.present && !s_imu2.present && s_ekf.initialized) {
        app_ekf_restart();               /* mất cả hai IMU: khi có lại sẽ khởi tạo từ đầu */
    }

    /* --- Từ kế im lặng quá lâu --- */
    if (s_mag.present && ((uint32_t)(now_ms - s_mag_last_ok_ms) > APP_MAG_TIMEOUT_MS)) {
        s_mag.present = false;
    }

    /* --- MAGCAL hết thời gian thu --- */
    if (s_magcal.active && ((uint32_t)(now_ms - s_magcal.start_ms) >= (uint32_t)CFG_MAGCAL_DURATION_MS)) {
        app_magcal_finish();
    }

    /* --- Dò lại thiết bị đang mất, mỗi giây thử 1 thiết bị --- */
    if (++s_reprobe_div >= APP_REPROBE_PERIOD) {
        s_reprobe_div = 0U;
        switch (s_reprobe_turn) {
        case 0:
            if (!s_imu1.present && MPU6500_Probe(&s_imu1, &s_bus, SENSORS_SPI_IMU1)) {
                (void)MPU6500_Init(&s_imu1, &s_bus, SENSORS_SPI_IMU1);
            }
            break;
        case 1:
            if (!s_imu2.present && MPU6500_Probe(&s_imu2, &s_bus, SENSORS_SPI_IMU2)) {
                (void)MPU6500_Init(&s_imu2, &s_bus, SENSORS_SPI_IMU2);
            }
            break;
        case 2:
            if (!s_mag.present && Mag_Init(&s_mag, &s_bus)) {
                s_mag_last_ok_ms = now_ms;
                s_mag_fail = 0U;
            }
            break;
        default:
            if (!s_ina.present && INA219_Init(&s_ina, &s_bus)) {
                s_ina_fail = 0U;
            }
            break;
        }
        s_reprobe_turn = (uint8_t)((s_reprobe_turn + 1U) & 0x03U);
    }

    /* --- Cập nhật bảng lỗi (LED FAULT) --- */
    SysMon_SetFault(SYS_FAULT_INA219,  !s_ina.present);
    SysMon_SetFault(SYS_FAULT_QMC5883, !s_mag.present);
    SysMon_SetFault(SYS_FAULT_MPU1,    !s_imu1.present);
    SysMon_SetFault(SYS_FAULT_MPU2,    !s_imu2.present);

    g_app_diag.imu1_ok   = s_imu1.present ? 1U : 0U;
    g_app_diag.imu2_ok   = s_imu2.present ? 1U : 0U;
    g_app_diag.mag_ok    = s_mag.present ? 1U : 0U;
    g_app_diag.ina_ok    = s_ina.present ? 1U : 0U;
    g_app_diag.ekf_ready = s_ekf.initialized ? 1U : 0U;
    g_app_diag.gyro_bias_rads[0] = s_ekf.bg[0];
    g_app_diag.gyro_bias_rads[1] = s_ekf.bg[1];
    g_app_diag.gyro_bias_rads[2] = s_ekf.bg[2];
}

/* -------------------------------------------------------------------------- */
/* Xử lý một mẫu IMU                                                          */
/* -------------------------------------------------------------------------- */

static void app_sample_step(uint32_t ts_us, uint32_t now_ms)
{
    /* Khoảng thời gian thật giữa hai mẫu (đo bằng thời điểm xung INT). */
    uint32_t dt_us = (uint32_t)(ts_us - s_last_ts_us);
    s_last_ts_us = ts_us;
    if (s_first_sample) {
        s_first_sample = false;
        dt_us = APP_SAMPLE_PERIOD_US;
    }
    if (dt_us < 200UL)   { dt_us = 200UL; }
    if (dt_us > 20000UL) { dt_us = 20000UL; }
    g_app_diag.dt_us_last = dt_us;
    float dt = (float)dt_us * 1.0e-6f;

    /* 1. Đọc hai IMU, đưa IMU2 về hệ trục IMU1, lấy trung bình. */
    imu_sample_t imu1;
    imu_sample_t imu2;
    imu_sample_t fused;
    imu1.valid = false;
    imu2.valid = false;
    if (s_imu1.present) {
        (void)MPU6500_Read(&s_imu1, &s_bus, &imu1);
    }
    if (s_imu2.present) {
        (void)MPU6500_Read(&s_imu2, &s_bus, &imu2);
        Align_IMU_Axes(&imu2, &imu2);
    }
    Sensors_AverageImu(&imu1, &imu2, &fused);

    /* 2. EKF nhịp nhanh. */
    if (fused.valid) {
        g_app_diag.accel_g[0] = fused.accel_g.x; g_app_diag.accel_g[1] = fused.accel_g.y; g_app_diag.accel_g[2] = fused.accel_g.z;
        if (s_ekf.initialized) {
            EKF_Predict(&s_ekf, &fused.gyro_rads, dt);
            EKF_AccumAccel(&s_ekf, &fused.accel_g);
        } else {
            app_ekf_init_step(&fused);
        }
    }

    /* 3. Một khe việc chậm. */
    switch (s_slot) {
    case 0:
        EKF_CovPropagate(&s_ekf);
        break;
    case 1:
        (void)EKF_UpdateAccel(&s_ekf);
        break;
    case 2:
        app_mag_step(now_ms);
        break;
    case 3:
        if (s_mag_fresh) {
            s_mag_fresh = false;
            (void)EKF_UpdateMag(&s_ekf, &s_mag_body);
        }
        break;
    default:
        app_ina_step();
        app_health_step(now_ms);
        break;
    }
    s_slot = (uint8_t)((s_slot + 1U) % APP_SLOT_COUNT);

    /* 4. Nạp số liệu mới nhất cho frame. */
    if (Comm_TelemetryWanted()) {
        comm_telemetry_t t;
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        if (s_ekf.initialized) {
            EKF_GetEulerDeg(&s_ekf, &roll, &pitch, &yaw);
        }
        g_app_diag.roll_deg = roll; g_app_diag.pitch_deg = pitch; g_app_diag.yaw_deg = yaw;

        t.roll_cdeg  = deg_to_cdeg(roll);
        t.pitch_cdeg = deg_to_cdeg(pitch);
        t.yaw_cdeg   = deg_to_cdeg(yaw);
        t.voltage_mv = clamp_u16(s_ina.bus_mv);
        t.current_ma = clamp_i16(s_ina.current_ma);
        t.power_mw   = clamp_u16((int32_t)INA219_PowerMw(&s_ina));
        Comm_UpdateTelemetry(&t);
    }

    g_app_diag.samples++;
}

/* -------------------------------------------------------------------------- */
/* API                                                                        */
/* -------------------------------------------------------------------------- */

void App_Init(void)
{
    static const comm_port_t comm_port = { BSP_UartTxStart, BSP_UartSetBaud };
    static const bsp_uart_callbacks_t uart_cb = { Comm_OnTxComplete, Comm_OnRxByte, app_uart_error_isr };

    memset(&g_app_diag, 0, sizeof(g_app_diag));

    /* Thứ tự quan trọng: Comm_Init trước khi nối callback UART (ngắt có thể tới ngay). */
    Comm_Init(&comm_port, (uint32_t)CFG_UART_DEFAULT_BAUD, app_on_user_cmd);
    BSP_UartSetCallbacks(&uart_cb);
    BSP_Init();

    /* Chờ cảm biến ổn định nguồn (MPU6500 cần tới 100 ms sau khi cấp điện). */
    BSP_DelayMs(100U);

    (void)MPU6500_Init(&s_imu1, &s_bus, SENSORS_SPI_IMU1);
    (void)MPU6500_Init(&s_imu2, &s_bus, SENSORS_SPI_IMU2);
    (void)Mag_Init(&s_mag, &s_bus);
    (void)INA219_Init(&s_ina, &s_bus);

    g_app_diag.imu1_whoami = s_imu1.whoami;
    g_app_diag.imu2_whoami = s_imu2.whoami;

    /* Bắt đầu đếm thời gian LED sau khi phần khởi tạo (chặn) đã xong. */
    SysMon_Init(app_led_write, BSP_Millis());
    SysMon_SetFault(SYS_FAULT_MPU1,    !s_imu1.present);
    SysMon_SetFault(SYS_FAULT_MPU2,    !s_imu2.present);
    SysMon_SetFault(SYS_FAULT_QMC5883, !s_mag.present);
    SysMon_SetFault(SYS_FAULT_INA219,  !s_ina.present);

    app_ekf_restart();

    uint32_t now_ms = BSP_Millis();
    s_last_ts_us       = BSP_Micros();
    s_last_drdy_ms     = now_ms;
    s_mag_last_ok_ms   = now_ms;
    s_mag_last_drdy_ms = now_ms;
    s_drdy_alive       = false;
    s_first_sample     = true;
    s_slot             = 0U;

    /* Bỏ các xung INT tích lũy trong lúc khởi tạo. */
    (void)BSP_ImuDrdyTake(BSP_IMU1, NULL);
    (void)BSP_ImuDrdyTake(BSP_IMU2, NULL);
    (void)BSP_MagDrdyTake();
}

void App_Loop(void)
{
    uint32_t now_ms = BSP_Millis();
    uint32_t now_us = BSP_Micros();

    BSP_Poll();

    /* --- Nhịp lấy mẫu: ưu tiên xung INT của IMU1 (hoặc IMU2 nếu IMU1 hỏng) --- */
    uint8_t  src   = (uint8_t)(s_imu1.present ? BSP_IMU1 : BSP_IMU2);
    uint8_t  other = (uint8_t)((src == BSP_IMU1) ? BSP_IMU2 : BSP_IMU1);
    uint32_t ts_us = now_us;
    uint32_t n     = BSP_ImuDrdyTake(src, &ts_us);
    g_app_diag.imu2_drdy += BSP_ImuDrdyTake(other, NULL);

    bool trigger = false;
    if (n > 0U) {
        trigger = true;
        s_drdy_alive = true;
        s_last_drdy_ms = now_ms;
        if (n > 1U) {
            g_app_diag.drdy_missed += (uint32_t)(n - 1U);    /* vòng lặp chậm hơn 1 ms */
        }
    } else {
        if (s_drdy_alive && ((uint32_t)(now_ms - s_last_drdy_ms) > APP_DRDY_TIMEOUT_MS)) {
            s_drdy_alive = false;
        }
        /* Chân INT không hoạt động (chưa bật EXTI trong CubeMX, IMU hỏng...):
         * tự lấy mẫu theo đồng hồ để hệ thống vẫn chạy. */
        if (!s_drdy_alive && ((uint32_t)(now_us - s_last_ts_us) >= APP_SAMPLE_PERIOD_US)) {
            trigger = true;
            ts_us = now_us;
        }
    }
    g_app_diag.drdy_alive = s_drdy_alive ? 1U : 0U;

    if (trigger) {
        app_sample_step(ts_us, now_ms);

        uint32_t used = (uint32_t)(BSP_Micros() - now_us);
        g_app_diag.step_us_last = used;
        if (used > g_app_diag.step_us_max) {
            g_app_diag.step_us_max = used;
        }
    }

    /* --- Truyền thông --- */
    Comm_Process(now_ms);
    if (Comm_TakeErrorFlags() != 0UL) {
        SysMon_RaiseTimed(SYS_FAULT_UART, (uint32_t)CFG_FAULT_EVENT_HOLD_MS);
    }

    /* --- LED --- */
    SysMon_Process(now_ms);
}
