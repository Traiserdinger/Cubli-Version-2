/**
 * @file    comm_protocol.h
 * @brief   Dataframe nhị phân, phân tích lệnh RX và máy trạng thái truyền UART.
 *
 * Module không gọi HAL: việc truyền DMA và đổi baudrate đi qua comm_port_t.
 *
 * ===========================================================================
 * 1. DATAFRAME (16 byte, số nhiều byte theo little-endian = byte thấp trước)
 * ===========================================================================
 *   Byte  Trường       Kiểu      Ý nghĩa
 *   ----  -----------  --------  -------------------------------------------
 *    0    HEADER0      0xAA      Đánh dấu đầu frame
 *    1    HEADER1      0x55
 *    2    SEQ          uint8     Số thứ tự, +1 mỗi frame, 255 -> 0 (phát hiện mất frame)
 *   3-4   ROLL         int16     Roll  x 100 (đơn vị 0.01 độ), -18000..18000
 *   5-6   PITCH        int16     Pitch x 100,                    -9000..9000
 *   7-8   YAW          int16     Yaw   x 100,                   -18000..18000
 *   9-10  VOLTAGE      uint16    Điện áp pin (mV)
 *  11-12  CURRENT      int16     Dòng tải (mA), dương = pin đang xả
 *  13-14  POWER        uint16    Công suất (mW)
 *   15    CRC8         uint8     CRC-8 của byte 0..14
 *                                (đa thức 0x07 = x^8+x^2+x+1, khởi tạo 0x00,
 *                                 MSB trước, không đảo bit, không XOR cuối)
 *
 * ===========================================================================
 * 2. TÍNH BAUDRATE
 * ===========================================================================
 *   UART 8N1: mỗi byte = 10 bit trên dây (1 start + 8 data + 1 stop).
 *   1 frame = 16 byte = 160 bit.
 *   Yêu cầu >= 1000 frame/s  ->  cần >= 160 000 bit/s.
 *
 *   Baud      Frame/s tối đa   Ghi chú
 *   -------   --------------   ----------------------------------------------
 *    57600        360          mặc định khi khởi động (không đủ 1000 frame/s)
 *   115200        720          chưa đủ
 *   230400       1440          đủ, dư 44 %
 *   460800       2880          ĐƯỢC CHỌN cho BURST - dư 2.88 lần
 *   921600       5760          sát giới hạn của IC cách ly
 *
 *   Giới hạn phần cứng: ADuM1201ARZ (hậu tố "A") chỉ bảo đảm tới 1 Mbps, độ
 *   rộng xung nhỏ nhất 1000 ns. Ở 460800 bps một bit dài 2170 ns (dư 2.17 lần),
 *   ở 921600 bps chỉ còn 1085 ns (dư 8 %). USART1 của STM32F103 chạy từ PCLK2 =
 *   72 MHz: BRR = 72e6/460800 = 156.25 -> 156 -> baud thực 461538 (sai +0.16 %).
 *   => Chọn 460800 bps: 1000 frame mất 16000 byte * 10 / 460800 = 0.347 s < 1 s.
 *
 * ===========================================================================
 * 3. CHẾ ĐỘ TRUYỀN
 * ===========================================================================
 *   STREAM (mặc định): mỗi khi có kết quả EKF mới và UART đang rảnh thì gửi 1
 *   frame. Tốc độ frame = min(1000 Hz, baud/160): 57600 -> 360 frame/s,
 *   >= 230400 -> 1000 frame/s. Không bao giờ dồn hàng đợi: luôn gửi dữ liệu
 *   mới nhất, mỗi bộ dữ liệu chỉ gửi một lần.
 *
 *   BURST (lệnh "BURST"):
 *     1. Gửi nốt frame đang dở, im lặng CFG_UART_GUARD_MS (20 ms).
 *     2. Đổi sang CFG_UART_BURST_BAUD (460800).
 *     3. Gửi liền CFG_BURST_FRAME_COUNT (1000) frame, frame sau nối ngay sau
 *        frame trước (nối chuỗi trong ngắt DMA/TC nên không phụ thuộc vòng lặp
 *        chính). Mỗi frame mang kết quả EKF mới nhất tại thời điểm đóng gói;
 *        EKF cập nhật 1000 lần/giây nên ở 2880 frame/s sẽ có ~3 frame liên tiếp
 *        cùng dữ liệu (SEQ vẫn tăng đều).
 *     4. Im lặng 20 ms, quay về baudrate trước đó, tiếp tục STREAM.
 *
 * ===========================================================================
 * 4. LỆNH RX (văn bản ASCII, kết thúc bằng CR, LF hoặc im lặng >= 30 ms)
 * ===========================================================================
 *   "9600" "19200" "38400" "57600" "115200" "230400" "460800" "921600"
 *              -> đổi baudrate USART1 (HAL_UART_DeInit + HAL_UART_Init). Lệnh
 *                 có hiệu lực sau khi frame đang gửi kết thúc + 20 ms im lặng.
 *   "BURST"    -> chạy chế độ BURST ở trên.
 *   Lệnh khác  -> chuyển cho hàm on_user_cmd (ví dụ "MAGCAL" do app xử lý).
 *   Không phân biệt hoa thường. Số không nằm trong danh sách bị bỏ qua.
 *
 * ===========================================================================
 * 5. HƯỚNG DẪN PARSE BÊN FPGA / PC
 * ===========================================================================
 *   Máy trạng thái nhận (mỗi byte UART nhận được):
 *     WAIT_AA : nếu byte == 0xAA -> crc = crc8_step(0, 0xAA); sang WAIT_55
 *     WAIT_55 : nếu byte == 0x55 -> crc = crc8_step(crc, 0x55); cnt = 2; sang DATA
 *               nếu byte == 0xAA -> ở lại WAIT_55 (crc = crc8_step(0, 0xAA))
 *               ngược lại        -> về WAIT_AA
 *     DATA    : buf[cnt] = byte; nếu cnt < 15: crc = crc8_step(crc, byte); cnt++
 *               nếu cnt == 15 (byte CRC): frame hợp lệ khi byte == crc;
 *               về WAIT_AA
 *   crc8_step cho 1 byte, viết dạng tổ hợp cho FPGA (c = crc XOR data):
 *     n[0] = c[0]^c[6]^c[7]        n[4] = c[2]^c[3]^c[4]
 *     n[1] = c[0]^c[1]^c[6]        n[5] = c[3]^c[4]^c[5]
 *     n[2] = c[0]^c[1]^c[2]^c[6]   n[6] = c[4]^c[5]^c[6]
 *     n[3] = c[1]^c[2]^c[3]^c[7]   n[7] = c[5]^c[6]^c[7]
 *   Ghép số 16 bit: giá_trị = byte_thấp | (byte_cao << 8); các trường int16 là
 *   số bù 2. Góc (độ) = giá_trị / 100.
 *   Xem Tools/frame_parser.v (Verilog tham khảo) và Tools/daktnc_monitor.py (PC).
 */
#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define COMM_FRAME_SIZE        16U
#define COMM_FRAME_HEADER0     0xAAU
#define COMM_FRAME_HEADER1     0x55U
#define COMM_BITS_PER_BYTE     10U       /* 8N1 */

/** Baudrate nhỏ nhất (bps) để truyền được frames_per_s frame mỗi giây. */
#define COMM_MIN_BAUD_FOR_RATE(frames_per_s) \
    ((uint32_t)(frames_per_s) * COMM_FRAME_SIZE * COMM_BITS_PER_BYTE)

/** Dữ liệu đo đưa vào frame. */
typedef struct {
    int16_t  roll_cdeg;       /**< 0.01 độ */
    int16_t  pitch_cdeg;
    int16_t  yaw_cdeg;
    uint16_t voltage_mv;
    int16_t  current_ma;
    uint16_t power_mw;
} comm_telemetry_t;

/** Trạng thái máy truyền. */
typedef enum {
    COMM_STATE_STREAM = 0,    /**< Truyền liên tục. */
    COMM_STATE_DRAIN,         /**< Chờ frame đang gửi kết thúc trước khi đổi baud. */
    COMM_STATE_GUARD,         /**< Khoảng lặng sau khi đổi baud. */
    COMM_STATE_BURST          /**< Đang dồn gửi N frame. */
} comm_state_t;

/** Cờ lỗi (bitmask) - lấy bằng Comm_TakeErrorFlags(). */
#define COMM_ERR_RX_OVERFLOW   0x01U     /**< Bộ đệm RX phần mềm bị tràn. */
#define COMM_ERR_UART_OVERRUN  0x02U     /**< UART báo overrun (ORE). */
#define COMM_ERR_DMA           0x04U     /**< Lỗi DMA khi truyền. */
#define COMM_ERR_TX_STALL      0x08U     /**< Truyền bị treo / không khởi động được. */
#define COMM_ERR_BAUD          0x10U     /**< Đổi baudrate thất bại. */

/** Các hàm phần cứng mà tầng BSP phải cung cấp. */
typedef struct {
    /** Bắt đầu truyền len byte bằng DMA, trả về ngay. data phải còn hợp lệ tới khi truyền xong. */
    bool (*tx_start)(const uint8_t *data, uint16_t len);
    /** Đổi baudrate (DeInit + Init), bật lại RX. Chỉ được gọi khi không đang truyền. */
    bool (*set_baud)(uint32_t baud);
} comm_port_t;

/** Hàm nhận các lệnh không thuộc giao thức (đã viết HOA, đã cắt khoảng trắng). */
typedef void (*comm_user_cmd_cb_t)(const char *line);

typedef struct {
    uint32_t frames_sent;
    uint32_t bursts_done;
    uint32_t last_burst_ms;      /**< Thời gian gửi 1000 frame của lần BURST gần nhất. */
    uint32_t cmd_ok;
    uint32_t cmd_bad;
    uint32_t rx_bytes;
} comm_stats_t;

/* ---------------- Gọi từ vòng lặp chính ---------------- */

void Comm_Init(const comm_port_t *port, uint32_t initial_baud, comm_user_cmd_cb_t on_user_cmd);

/** Nạp kết quả đo mới nhất (không chặn, an toàn với ngắt). */
void Comm_UpdateTelemetry(const comm_telemetry_t *t);

/** true nếu module đang ở trạng thái có gửi frame (STREAM/BURST) - khi false
 *  thì chưa cần tính số liệu mới. */
bool Comm_TelemetryWanted(void);

/** Chạy máy trạng thái: xử lý lệnh RX, gửi frame, điều phối đổi baud và BURST. */
void Comm_Process(uint32_t now_ms);

/** Xếp một dòng văn bản (<= 95 ký tự) để gửi xen giữa các frame. false nếu đang bận. */
bool Comm_SendText(const char *text);

uint32_t     Comm_GetBaud(void);
comm_state_t Comm_GetState(void);
void         Comm_GetStats(comm_stats_t *out);

/** Trả về các cờ COMM_ERR_* đã xảy ra từ lần gọi trước rồi xóa chúng. */
uint32_t Comm_TakeErrorFlags(void);

/* ---------------- Gọi từ ngắt (qua tầng BSP) ---------------- */

/** Truyền DMA vừa xong (HAL_UART_TxCpltCallback). */
void Comm_OnTxComplete(void);

/** Nhận được 1 byte (HAL_UART_RxCpltCallback). */
void Comm_OnRxByte(uint8_t byte);

/** UART/DMA báo lỗi. flags = tổ hợp COMM_ERR_*; tx_aborted = true nếu lần truyền bị hủy. */
void Comm_OnUartError(uint32_t flags, bool tx_aborted);

/* ---------------- Hàm thuần (dùng được ở mọi nơi, kể cả PC) ---------------- */

/** CRC-8 đa thức 0x07, khởi tạo 0x00. */
uint8_t Comm_Crc8(const uint8_t *data, uint16_t len);

/** Đóng gói 1 frame 16 byte. */
void Comm_PackFrame(uint8_t out[COMM_FRAME_SIZE], uint8_t seq, const comm_telemetry_t *t);

/** Giải mã + kiểm tra 1 frame 16 byte. false nếu sai header hoặc CRC. */
bool Comm_UnpackFrame(const uint8_t in[COMM_FRAME_SIZE], uint8_t *seq, comm_telemetry_t *t);

/** true nếu baud nằm trong danh sách cho phép. */
bool Comm_IsBaudSupported(uint32_t baud);

#ifdef __cplusplus
}
#endif

#endif /* COMM_PROTOCOL_H */
