/**
 * @file    app_config.h
 * @brief   TẤT CẢ thông số cần chỉnh của firmware nằm ở file này.
 *
 * Mạch: DAKTNC_Nhom2 - STM32F103C8T6 + 2x MPU6500/9250 (SPI) + QMC5883L (I2C)
 *       + INA219 (I2C) + ADuM1201ARZ (UART cách ly sang FPGA).
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ========================================================================== */
/* 1. IMU (MPU6500 / MPU9250)                                                 */
/* ========================================================================== */

/** Tần số lấy mẫu IMU (Hz). 1000 = mỗi 1 ms có 1 mẫu -> đủ cho 1000 frame/s. */
#define CFG_IMU_ODR_HZ                  1000U

/** Thang đo gyro: 0=±250, 1=±500, 2=±1000, 3=±2000 độ/giây. */
#define CFG_IMU_GYRO_FS_SEL             3U

/** Thang đo gia tốc: 0=±2g, 1=±4g, 2=±8g, 3=±16g. ±8g để rung không bão hòa. */
#define CFG_IMU_ACCEL_FS_SEL            2U

/** Lọc thông thấp số trong chip (DLPF) cho gyro:
 *  1=184Hz, 2=92Hz, 3=41Hz, 4=20Hz, 5=10Hz. Càng thấp càng sạch rung nhưng trễ. */
#define CFG_IMU_GYRO_DLPF               3U

/** DLPF cho gia tốc: 1=184Hz, 2=92Hz, 3=41Hz, 4=20Hz, 5=10Hz. */
#define CFG_IMU_ACCEL_DLPF              3U

/* ========================================================================== */
/* 2. Từ kế (QMC5883L, tự nhận HMC5883L nếu gắn chip đó)                      */
/* ========================================================================== */

/**
 * Ma trận đưa trục từ kế về hệ trục IMU1 (X sang phải, Y lên trên, Z ra khỏi
 * mặt mạch):  body = ALIGN * mag_chip.
 * MẶC ĐỊNH là ma trận đơn vị (giả sử U11 đặt cùng hướng U8). PHẢI kiểm tra lại
 * theo hướng đặt chip trên PCB - xem mục "Kiểm tra hướng từ kế" trong README.
 */
#define CFG_MAG_ALIGN_XX                (1.0f)
#define CFG_MAG_ALIGN_XY                (0.0f)
#define CFG_MAG_ALIGN_XZ                (0.0f)
#define CFG_MAG_ALIGN_YX                (0.0f)
#define CFG_MAG_ALIGN_YY                (1.0f)
#define CFG_MAG_ALIGN_YZ                (0.0f)
#define CFG_MAG_ALIGN_ZX                (0.0f)
#define CFG_MAG_ALIGN_ZY                (0.0f)
#define CFG_MAG_ALIGN_ZZ                (1.0f)

/**
 * Hiệu chuẩn từ kế (hard-iron offset tính bằng gauss, soft-iron scale).
 * Gửi lệnh "MAGCAL" qua UART, xoay mạch đủ mọi hướng trong 20 giây, firmware
 * in ra một dòng kết quả -> chép số vào đây để lưu vĩnh viễn.
 */
#define CFG_MAG_OFFSET_X                (0.0f)
#define CFG_MAG_OFFSET_Y                (0.0f)
#define CFG_MAG_OFFSET_Z                (0.0f)
#define CFG_MAG_SCALE_X                 (1.0f)
#define CFG_MAG_SCALE_Y                 (1.0f)
#define CFG_MAG_SCALE_Z                 (1.0f)

/** Thời gian thu dữ liệu cho lệnh MAGCAL (ms). */
#define CFG_MAGCAL_DURATION_MS          20000U

/** Độ từ thiên (rad, dương về phía Đông). 0 = yaw tính theo Bắc từ. */
#define CFG_MAG_DECLINATION_RAD         (0.0f)

/* ========================================================================== */
/* 3. INA219                                                                  */
/* ========================================================================== */

/** Địa chỉ I2C 7-bit. Sơ đồ nối A0=A1=GND -> 0x40. */
#define CFG_INA219_ADDR                 0x40U

/** Giá trị điện trở shunt R13 (mΩ). Sơ đồ là 0.01Ω = 10. Nếu đã đổi sang
 *  0.05Ω thì sửa thành 50. */
#define CFG_INA219_SHUNT_MILLIOHM       10U

/** Dòng lớn nhất cần đo (mA) - dùng để tự chọn dải PGA của INA219. */
#define CFG_INA219_MAX_CURRENT_MA       1600U

/* ========================================================================== */
/* 4. UART / Dataframe                                                        */
/* ========================================================================== */

/** Baudrate mặc định khi khởi động (yêu cầu đề bài). */
#define CFG_UART_DEFAULT_BAUD           57600U

/**
 * Baudrate dùng cho chế độ BURST - xem phần tính toán trong comm_protocol.h.
 * Tóm tắt: frame 16 byte = 160 bit trên dây (8N1) -> 1000 frame/s cần tối
 * thiểu 160 000 bps. ADuM1201ARZ (hậu tố A) chỉ bảo đảm tới 1 Mbps (xung hẹp
 * nhất 1 µs). 460800 bps: bit dài 2.17 µs (dư 2.17 lần so với giới hạn cách
 * ly), tải được 2880 frame/s (dư 2.88 lần so với yêu cầu). Muốn nhanh hơn có
 * thể đặt 921600 (sát giới hạn 1 Mbps của ADuM1201ARZ).
 */
#define CFG_UART_BURST_BAUD             460800U

/** Số frame của một lần BURST. */
#define CFG_BURST_FRAME_COUNT           1000U

/** Sau khi BURST xong: 1 = tự quay về baudrate trước đó, 0 = ở lại baud cao. */
#define CFG_BURST_RETURN_TO_PREV_BAUD   1

/** Khoảng lặng (ms) trước và sau mỗi lần đổi baudrate để phía FPGA kịp đổi theo. */
#define CFG_UART_GUARD_MS               20U

/* ========================================================================== */
/* 5. EKF                                                                     */
/* ========================================================================== */

/** Mật độ nhiễu gyro dùng trong mô hình (rad/s/√Hz). Tăng -> tin gia tốc/từ kế
 *  hơn (bám nhanh, nhiễu hơn). Giảm -> tin gyro hơn (mượt, chống rung tốt). */
#define CFG_EKF_SIGMA_GYRO              (3.0e-3f)

/** Random-walk của bias gyro (rad/s/√s). */
#define CFG_EKF_SIGMA_BIAS              (2.0e-4f)

/** Độ lệch chuẩn hướng trọng lực đo từ gia tốc (đơn vị ~rad) khi đứng yên. */
#define CFG_EKF_SIGMA_ACCEL             (0.05f)

/** Hệ số "phạt" khi |a| lệch khỏi 1g (rung / gia tốc tuyến tính). */
#define CFG_EKF_ACCEL_DEV_GAIN          (5.0f)

/** Bỏ hẳn phép đo gia tốc khi | |a|/g - 1 | vượt ngưỡng này. */
#define CFG_EKF_ACCEL_REJECT_DEV        (0.5f)

/** Độ lệch chuẩn góc yaw đo từ từ kế (rad). */
#define CFG_EKF_SIGMA_MAG               (0.05f)

/** Hệ số "phạt" và ngưỡng loại khi độ lớn từ trường lệch khỏi giá trị tham chiếu. */
#define CFG_EKF_MAG_DEV_GAIN            (3.0f)
#define CFG_EKF_MAG_REJECT_DEV          (0.35f)

/** Giới hạn bias gyro hợp lệ (rad/s). Vượt -> báo lỗi EKF. */
#define CFG_EKF_BIAS_LIMIT              (0.35f)

/** Số mẫu IMU dùng để khởi tạo tư thế ban đầu (mạch cần đứng yên). */
#define CFG_EKF_INIT_SAMPLES            300U

/* ========================================================================== */
/* 6. LED                                                                     */
/* ========================================================================== */

#define CFG_LED_HEARTBEAT_PERIOD_MS     1000U   /* 1 chu kỳ sáng-tắt = 1000 ms */
#define CFG_LED_FAULT_ON_MS             200U
#define CFG_LED_FAULT_OFF_MS            550U
#define CFG_LED_FAULT_PAUSE_MS          2000U

/**
 * Thứ tự ưu tiên khi có nhiều lỗi cùng lúc:
 * 0 = mã lỗi LỚN đè mã nhỏ (EKF > UART > MPU2 > MPU1 > QMC > INA219) - mặc định,
 *     vì các lỗi mã lớn làm hỏng dữ liệu góc nặng hơn.
 * 1 = mã lỗi NHỎ đè mã lớn.
 */
#define CFG_FAULT_PRIORITY_LOW_CODE_FIRST 0

/** Thời gian giữ các lỗi dạng "sự kiện" (UART, EKF) trên LED sau khi xảy ra. */
#define CFG_FAULT_EVENT_HOLD_MS         10000U

#endif /* APP_CONFIG_H */
