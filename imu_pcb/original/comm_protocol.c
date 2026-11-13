/**
 * @file    comm_protocol.c
 * @brief   Dataframe, lệnh RX, máy trạng thái STREAM / đổi baud / BURST.
 *
 * Phân chia ngữ cảnh:
 *   - Vòng lặp chính : Comm_Process(), Comm_UpdateTelemetry(), Comm_SendText()
 *   - Ngắt           : Comm_OnTxComplete(), Comm_OnRxByte(), Comm_OnUartError()
 * Dữ liệu chia sẻ giữa hai ngữ cảnh chỉ gồm các biến 1 byte/1 word ghi nguyên
 * tử và một bộ đệm kép cho số liệu đo, nên không cần tắt ngắt.
 */
#include "comm_protocol.h"
#include "app_config.h"

#include <string.h>

/* -------------------------------------------------------------------------- */
/* Hằng số                                                                    */
/* -------------------------------------------------------------------------- */

#define COMM_RX_RING_SIZE      64U       /* phải là lũy thừa của 2 */
#define COMM_LINE_MAX          16U       /* lệnh dài nhất: 15 ký tự */
#define COMM_TEXT_MAX          96U
#define COMM_LINE_IDLE_MS      30U       /* im lặng bao lâu thì coi như hết lệnh */
#define COMM_TX_STALL_MS       500U      /* quá lâu không truyền xong -> coi như treo */

/** Danh sách baudrate được phép (lệnh số khác sẽ bị bỏ qua để nhiễu trên
 *  đường RX không làm mất liên lạc). */
static const uint32_t k_baud_list[] = {
    9600UL, 19200UL, 38400UL, 57600UL, 115200UL, 230400UL, 460800UL, 921600UL
};

/** Bảng CRC-8 đa thức 0x07 (x^8 + x^2 + x + 1), khởi tạo 0x00. */
static const uint8_t k_crc8_table[256] = {
    0x00U, 0x07U, 0x0EU, 0x09U, 0x1CU, 0x1BU, 0x12U, 0x15U, 0x38U, 0x3FU, 0x36U, 0x31U,
    0x24U, 0x23U, 0x2AU, 0x2DU, 0x70U, 0x77U, 0x7EU, 0x79U, 0x6CU, 0x6BU, 0x62U, 0x65U,
    0x48U, 0x4FU, 0x46U, 0x41U, 0x54U, 0x53U, 0x5AU, 0x5DU, 0xE0U, 0xE7U, 0xEEU, 0xE9U,
    0xFCU, 0xFBU, 0xF2U, 0xF5U, 0xD8U, 0xDFU, 0xD6U, 0xD1U, 0xC4U, 0xC3U, 0xCAU, 0xCDU,
    0x90U, 0x97U, 0x9EU, 0x99U, 0x8CU, 0x8BU, 0x82U, 0x85U, 0xA8U, 0xAFU, 0xA6U, 0xA1U,
    0xB4U, 0xB3U, 0xBAU, 0xBDU, 0xC7U, 0xC0U, 0xC9U, 0xCEU, 0xDBU, 0xDCU, 0xD5U, 0xD2U,
    0xFFU, 0xF8U, 0xF1U, 0xF6U, 0xE3U, 0xE4U, 0xEDU, 0xEAU, 0xB7U, 0xB0U, 0xB9U, 0xBEU,
    0xABU, 0xACU, 0xA5U, 0xA2U, 0x8FU, 0x88U, 0x81U, 0x86U, 0x93U, 0x94U, 0x9DU, 0x9AU,
    0x27U, 0x20U, 0x29U, 0x2EU, 0x3BU, 0x3CU, 0x35U, 0x32U, 0x1FU, 0x18U, 0x11U, 0x16U,
    0x03U, 0x04U, 0x0DU, 0x0AU, 0x57U, 0x50U, 0x59U, 0x5EU, 0x4BU, 0x4CU, 0x45U, 0x42U,
    0x6FU, 0x68U, 0x61U, 0x66U, 0x73U, 0x74U, 0x7DU, 0x7AU, 0x89U, 0x8EU, 0x87U, 0x80U,
    0x95U, 0x92U, 0x9BU, 0x9CU, 0xB1U, 0xB6U, 0xBFU, 0xB8U, 0xADU, 0xAAU, 0xA3U, 0xA4U,
    0xF9U, 0xFEU, 0xF7U, 0xF0U, 0xE5U, 0xE2U, 0xEBU, 0xECU, 0xC1U, 0xC6U, 0xCFU, 0xC8U,
    0xDDU, 0xDAU, 0xD3U, 0xD4U, 0x69U, 0x6EU, 0x67U, 0x60U, 0x75U, 0x72U, 0x7BU, 0x7CU,
    0x51U, 0x56U, 0x5FU, 0x58U, 0x4DU, 0x4AU, 0x43U, 0x44U, 0x19U, 0x1EU, 0x17U, 0x10U,
    0x05U, 0x02U, 0x0BU, 0x0CU, 0x21U, 0x26U, 0x2FU, 0x28U, 0x3DU, 0x3AU, 0x33U, 0x34U,
    0x4EU, 0x49U, 0x40U, 0x47U, 0x52U, 0x55U, 0x5CU, 0x5BU, 0x76U, 0x71U, 0x78U, 0x7FU,
    0x6AU, 0x6DU, 0x64U, 0x63U, 0x3EU, 0x39U, 0x30U, 0x37U, 0x22U, 0x25U, 0x2CU, 0x2BU,
    0x06U, 0x01U, 0x08U, 0x0FU, 0x1AU, 0x1DU, 0x14U, 0x13U, 0xAEU, 0xA9U, 0xA0U, 0xA7U,
    0xB2U, 0xB5U, 0xBCU, 0xBBU, 0x96U, 0x91U, 0x98U, 0x9FU, 0x8AU, 0x8DU, 0x84U, 0x83U,
    0xDEU, 0xD9U, 0xD0U, 0xD7U, 0xC2U, 0xC5U, 0xCCU, 0xCBU, 0xE6U, 0xE1U, 0xE8U, 0xEFU,
    0xFAU, 0xFDU, 0xF4U, 0xF3U
};

/* -------------------------------------------------------------------------- */
/* Trạng thái nội bộ                                                          */
/* -------------------------------------------------------------------------- */

typedef enum {
    AFTER_GUARD_STREAM = 0,
    AFTER_GUARD_BURST
} after_guard_t;

typedef struct {
    comm_port_t        port;
    comm_user_cmd_cb_t user_cb;

    /* Số liệu đo: bộ đệm kép. Vòng lặp chính ghi vào ô không dùng rồi đổi chỉ
     * số; ngắt chỉ đọc ô đang dùng -> luôn đọc được một bộ số liệu nhất quán. */
    comm_telemetry_t telem[2];
    volatile uint8_t telem_idx;
    volatile uint8_t telem_new;

    /* Truyền */
    uint8_t          tx_frame[COMM_FRAME_SIZE];
    volatile uint8_t tx_busy;
    volatile uint8_t seq;
    char             text_buf[COMM_TEXT_MAX];
    uint8_t          text_len;
    volatile uint8_t text_pending;
    uint32_t         tx_watch_ms;        /* lần cuối thấy việc truyền có tiến triển */
    uint32_t         tx_watch_frames;

    /* Máy trạng thái */
    volatile uint8_t  state;             /* comm_state_t */
    after_guard_t     after_guard;
    uint32_t          guard_start_ms;
    uint32_t          cur_baud;
    uint32_t          saved_baud;
    volatile uint32_t pending_baud;
    volatile uint8_t  pending_burst;
    volatile uint32_t burst_remaining;   /* số frame của đợt BURST chưa bắt đầu gửi */
    uint32_t          burst_start_ms;

    /* Nhận */
    volatile uint8_t rx_ring[COMM_RX_RING_SIZE];
    volatile uint8_t rx_head;            /* ngắt ghi */
    volatile uint8_t rx_tail;            /* vòng lặp chính ghi */
    char             line[COMM_LINE_MAX];
    uint8_t          line_len;
    bool             line_invalid;
    uint32_t         last_rx_ms;

    /* Lỗi: mỗi cờ 1 byte để ngắt đặt và vòng lặp chính xóa không tranh chấp. */
    volatile uint8_t err_rx_overflow;
    volatile uint8_t err_overrun;
    volatile uint8_t err_dma;
    volatile uint8_t err_stall;
    volatile uint8_t err_baud;

    volatile uint32_t frames_sent;
    comm_stats_t      stats;
} comm_ctx_t;

static comm_ctx_t s;

/* -------------------------------------------------------------------------- */
/* Hàm thuần                                                                  */
/* -------------------------------------------------------------------------- */

uint8_t Comm_Crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00U;
    for (uint16_t i = 0U; i < len; i++) {
        crc = k_crc8_table[crc ^ data[i]];
    }
    return crc;
}

static void put_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)(v >> 8);
}

static uint16_t get_u16le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

void Comm_PackFrame(uint8_t out[COMM_FRAME_SIZE], uint8_t seq, const comm_telemetry_t *t)
{
    out[0] = COMM_FRAME_HEADER0;
    out[1] = COMM_FRAME_HEADER1;
    out[2] = seq;
    put_u16le(&out[3],  (uint16_t)t->roll_cdeg);
    put_u16le(&out[5],  (uint16_t)t->pitch_cdeg);
    put_u16le(&out[7],  (uint16_t)t->yaw_cdeg);
    put_u16le(&out[9],  t->voltage_mv);
    put_u16le(&out[11], (uint16_t)t->current_ma);
    put_u16le(&out[13], t->power_mw);
    out[15] = Comm_Crc8(out, (uint16_t)(COMM_FRAME_SIZE - 1U));
}

bool Comm_UnpackFrame(const uint8_t in[COMM_FRAME_SIZE], uint8_t *seq, comm_telemetry_t *t)
{
    if ((in[0] != COMM_FRAME_HEADER0) || (in[1] != COMM_FRAME_HEADER1)) {
        return false;
    }
    if (Comm_Crc8(in, (uint16_t)(COMM_FRAME_SIZE - 1U)) != in[15]) {
        return false;
    }
    if (seq != NULL) {
        *seq = in[2];
    }
    if (t != NULL) {
        t->roll_cdeg  = (int16_t)get_u16le(&in[3]);
        t->pitch_cdeg = (int16_t)get_u16le(&in[5]);
        t->yaw_cdeg   = (int16_t)get_u16le(&in[7]);
        t->voltage_mv = get_u16le(&in[9]);
        t->current_ma = (int16_t)get_u16le(&in[11]);
        t->power_mw   = get_u16le(&in[13]);
    }
    return true;
}

bool Comm_IsBaudSupported(uint32_t baud)
{
    for (size_t i = 0U; i < (sizeof(k_baud_list) / sizeof(k_baud_list[0])); i++) {
        if (k_baud_list[i] == baud) {
            return true;
        }
    }
    return false;
}

/* -------------------------------------------------------------------------- */
/* Truyền                                                                     */
/* -------------------------------------------------------------------------- */

/**
 * Đóng gói số liệu mới nhất và bắt đầu truyền 1 frame.
 * Được gọi từ vòng lặp chính (khi đường truyền đang rảnh) hoặc từ ngắt truyền
 * xong (nối frame kế tiếp) - hai nơi không bao giờ chạy đồng thời: vòng lặp
 * chính chỉ gọi khi tx_busy == 0, tức là không còn ngắt truyền xong nào sắp tới.
 */
static bool comm_send_frame(void)
{
    const comm_telemetry_t *t = &s.telem[s.telem_idx];
    Comm_PackFrame(s.tx_frame, s.seq, t);

    s.tx_busy = 1U;                      /* đặt trước: ngắt xong có thể đến ngay */
    if (!s.port.tx_start(s.tx_frame, (uint16_t)COMM_FRAME_SIZE)) {
        s.tx_busy = 0U;
        return false;
    }
    s.seq = (uint8_t)(s.seq + 1U);
    s.frames_sent++;
    return true;
}

/** Gửi 1 frame của đợt BURST nếu còn. */
static void comm_burst_send_next(void)
{
    if (s.burst_remaining > 0U) {
        s.burst_remaining--;             /* giữ chỗ trước khi phát */
        if (!comm_send_frame()) {
            s.burst_remaining++;         /* phát không được: vòng lặp chính sẽ thử lại */
        }
    }
}

static void comm_apply_baud(uint32_t baud)
{
    if (baud == s.cur_baud) {
        return;
    }
    if (s.port.set_baud(baud)) {
        s.cur_baud = baud;
    } else {
        s.err_baud = 1U;
        /* Thử quay lại baud cũ để không mất liên lạc. */
        (void)s.port.set_baud(s.cur_baud);
    }
}

static void comm_enter_guard(uint32_t now_ms, after_guard_t after)
{
    s.after_guard    = after;
    s.guard_start_ms = now_ms;
    s.state          = (uint8_t)COMM_STATE_GUARD;
}

/* -------------------------------------------------------------------------- */
/* Nhận và phân tích lệnh                                                     */
/* -------------------------------------------------------------------------- */

static void comm_handle_line(void)
{
    /* Cắt khoảng trắng hai đầu. */
    uint8_t start = 0U;
    uint8_t end   = s.line_len;
    while ((start < end) && (s.line[start] == ' ')) { start++; }
    while ((end > start) && (s.line[end - 1U] == ' ')) { end--; }
    uint8_t len = (uint8_t)(end - start);

    if (!s.line_invalid && (len > 0U)) {
        const char *cmd = &s.line[start];
        s.line[end] = '\0';              /* line[] luôn còn 1 ô trống cho '\0' */

        /* Toàn chữ số -> lệnh đổi baudrate. */
        bool all_digits = true;
        uint32_t value = 0UL;
        for (uint8_t i = 0U; i < len; i++) {
            if ((cmd[i] < '0') || (cmd[i] > '9')) {
                all_digits = false;
                break;
            }
            value = (uint32_t)(value * 10U) + (uint32_t)(cmd[i] - '0');
        }

        if (all_digits) {
            if ((len <= 7U) && Comm_IsBaudSupported(value)) {
                s.pending_baud = value;
                s.stats.cmd_ok++;
            } else {
                s.stats.cmd_bad++;
            }
        } else if (strcmp(cmd, "BURST") == 0) {
            s.pending_burst = 1U;
            s.stats.cmd_ok++;
        } else if (s.user_cb != NULL) {
            s.user_cb(cmd);
            s.stats.cmd_ok++;
        } else {
            s.stats.cmd_bad++;
        }
    } else if (s.line_invalid) {
        s.stats.cmd_bad++;
    } else {
        /* dòng trống: bỏ qua */
    }
    s.line_len = 0U;
    s.line_invalid = false;
}

static void comm_process_rx(uint32_t now_ms)
{
    while (s.rx_tail != s.rx_head) {
        uint8_t c = s.rx_ring[s.rx_tail];
        s.rx_tail = (uint8_t)((s.rx_tail + 1U) & (COMM_RX_RING_SIZE - 1U));
        s.last_rx_ms = now_ms;
        s.stats.rx_bytes++;

        if ((c == (uint8_t)'\r') || (c == (uint8_t)'\n') || (c == 0U)) {
            comm_handle_line();
        } else if ((c >= 0x20U) && (c < 0x7FU)) {
            if ((c >= (uint8_t)'a') && (c <= (uint8_t)'z')) {
                c = (uint8_t)(c - 32U);  /* đổi sang chữ hoa */
            }
            if (s.line_len < (COMM_LINE_MAX - 1U)) {
                s.line[s.line_len++] = (char)c;
            } else {
                s.line_invalid = true;   /* quá dài */
            }
        } else {
            s.line_invalid = true;       /* byte rác (nhiễu, lệch baud) */
        }
    }

    /* Lệnh không có ký tự kết thúc: im lặng đủ lâu thì coi như hết lệnh. */
    if (((s.line_len > 0U) || s.line_invalid) &&
        ((uint32_t)(now_ms - s.last_rx_ms) >= COMM_LINE_IDLE_MS)) {
        comm_handle_line();
    }
}

/* -------------------------------------------------------------------------- */
/* API vòng lặp chính                                                         */
/* -------------------------------------------------------------------------- */

void Comm_Init(const comm_port_t *port, uint32_t initial_baud, comm_user_cmd_cb_t on_user_cmd)
{
    memset(&s, 0, sizeof(s));
    s.port     = *port;
    s.user_cb  = on_user_cmd;
    s.cur_baud = initial_baud;
    s.state    = (uint8_t)COMM_STATE_STREAM;
}

void Comm_UpdateTelemetry(const comm_telemetry_t *t)
{
    uint8_t spare = (uint8_t)(1U - s.telem_idx);
    s.telem[spare] = *t;
    s.telem_idx = spare;                 /* đổi ô: ghi 1 byte, nguyên tử */
    s.telem_new = 1U;
}

bool Comm_TelemetryWanted(void)
{
    /* Trong lúc chờ đổi baud / khoảng lặng không có frame nào được gửi. */
    return (s.state == (uint8_t)COMM_STATE_STREAM) || (s.state == (uint8_t)COMM_STATE_BURST);
}

bool Comm_SendText(const char *text)
{
    if (s.text_pending != 0U) {
        return false;
    }
    size_t n = strlen(text);
    if (n > (COMM_TEXT_MAX - 1U)) {
        n = COMM_TEXT_MAX - 1U;
    }
    memcpy(s.text_buf, text, n);
    s.text_len = (uint8_t)n;
    s.text_pending = (n > 0U) ? 1U : 0U;
    return true;
}

void Comm_Process(uint32_t now_ms)
{
    comm_process_rx(now_ms);

    /* Giám sát treo truyền: tx_busy mà quá lâu không có ngắt truyền xong. */
    if (s.tx_busy == 0U) {
        s.tx_watch_ms = now_ms;
    } else if (s.tx_watch_frames != s.frames_sent) {
        s.tx_watch_frames = s.frames_sent;
        s.tx_watch_ms = now_ms;
    } else if ((uint32_t)(now_ms - s.tx_watch_ms) >= COMM_TX_STALL_MS) {
        s.tx_busy = 0U;                  /* gỡ treo, lần sau thử truyền lại */
        s.err_stall = 1U;
        s.tx_watch_ms = now_ms;
    } else {
        /* đang truyền bình thường */
    }

    switch ((comm_state_t)s.state) {
    case COMM_STATE_STREAM:
        if ((s.pending_baud != 0UL) || (s.pending_burst != 0U)) {
            s.state = (uint8_t)COMM_STATE_DRAIN;      /* ngừng phát frame mới */
        } else if (s.tx_busy == 0U) {
            if (s.text_pending != 0U) {
                s.text_pending = 0U;
                s.tx_busy = 1U;
                if (!s.port.tx_start((const uint8_t *)s.text_buf, s.text_len)) {
                    s.tx_busy = 0U;
                    s.err_stall = 1U;
                }
            } else if (s.telem_new != 0U) {
                s.telem_new = 0U;
                if (!comm_send_frame()) {
                    s.err_stall = 1U;
                }
            } else {
                /* chưa có số liệu mới */
            }
        } else {
            /* đang truyền */
        }
        break;

    case COMM_STATE_DRAIN:
        if (s.tx_busy == 0U) {
            if (s.pending_baud != 0UL) {
                uint32_t baud = s.pending_baud;
                s.pending_baud = 0UL;
                comm_apply_baud(baud);
                comm_enter_guard(now_ms, AFTER_GUARD_STREAM);
            } else if (s.pending_burst != 0U) {
                s.pending_burst = 0U;
                s.saved_baud = s.cur_baud;
                comm_apply_baud((uint32_t)CFG_UART_BURST_BAUD);
                comm_enter_guard(now_ms, AFTER_GUARD_BURST);
            } else {
                s.state = (uint8_t)COMM_STATE_STREAM;
            }
        }
        break;

    case COMM_STATE_GUARD:
        /* +1 vì bộ đếm mili-giây có thể vừa sắp nhảy lúc bắt đầu: bảo đảm
         * khoảng lặng thật sự không ngắn hơn CFG_UART_GUARD_MS. */
        if ((uint32_t)(now_ms - s.guard_start_ms) >= ((uint32_t)CFG_UART_GUARD_MS + 1U)) {
            if (s.after_guard == AFTER_GUARD_BURST) {
                s.burst_remaining = (uint32_t)CFG_BURST_FRAME_COUNT;
                s.burst_start_ms  = now_ms;
                s.state = (uint8_t)COMM_STATE_BURST;
                comm_burst_send_next();              /* frame đầu; các frame sau do ngắt nối tiếp */
            } else {
                s.state = (uint8_t)COMM_STATE_STREAM;
            }
        }
        break;

    case COMM_STATE_BURST:
        if (s.tx_busy == 0U) {
            if (s.burst_remaining == 0U) {
                /* Xong đợt BURST. */
                s.stats.last_burst_ms = (uint32_t)(now_ms - s.burst_start_ms);
                s.stats.bursts_done++;
#if CFG_BURST_RETURN_TO_PREV_BAUD
                comm_apply_baud(s.saved_baud);
#endif
                comm_enter_guard(now_ms, AFTER_GUARD_STREAM);
            } else {
                /* Chuỗi nối trong ngắt bị đứt (HAL bận / lỗi): phát lại từ đây. */
                comm_burst_send_next();
            }
        }
        break;

    default:
        s.state = (uint8_t)COMM_STATE_STREAM;
        break;
    }
}

uint32_t Comm_GetBaud(void)
{
    return s.cur_baud;
}

comm_state_t Comm_GetState(void)
{
    return (comm_state_t)s.state;
}

void Comm_GetStats(comm_stats_t *out)
{
    *out = s.stats;
    out->frames_sent = s.frames_sent;
}

uint32_t Comm_TakeErrorFlags(void)
{
    uint32_t f = 0UL;
    if (s.err_rx_overflow != 0U) { s.err_rx_overflow = 0U; f |= COMM_ERR_RX_OVERFLOW; }
    if (s.err_overrun != 0U)     { s.err_overrun = 0U;     f |= COMM_ERR_UART_OVERRUN; }
    if (s.err_dma != 0U)         { s.err_dma = 0U;         f |= COMM_ERR_DMA; }
    if (s.err_stall != 0U)       { s.err_stall = 0U;       f |= COMM_ERR_TX_STALL; }
    if (s.err_baud != 0U)        { s.err_baud = 0U;        f |= COMM_ERR_BAUD; }
    return f;
}

/* -------------------------------------------------------------------------- */
/* API ngắt                                                                   */
/* -------------------------------------------------------------------------- */

void Comm_OnTxComplete(void)
{
    s.tx_busy = 0U;
    if (s.state == (uint8_t)COMM_STATE_BURST) {
        comm_burst_send_next();          /* nối ngay frame kế tiếp, không chờ vòng lặp chính */
    } else if ((s.state == (uint8_t)COMM_STATE_STREAM) && (s.telem_new != 0U) &&
               (s.text_pending == 0U) && (s.pending_baud == 0UL) && (s.pending_burst == 0U)) {
        /* STREAM ở baud thấp: đã có số liệu mới chờ sẵn -> phát luôn trong ngắt
         * để đạt đúng tốc độ tối đa của đường truyền (57600 bps -> 360 frame/s). */
        s.telem_new = 0U;
        (void)comm_send_frame();
    } else {
        /* không có gì để gửi ngay: vòng lặp chính sẽ lo */
    }
}

void Comm_OnRxByte(uint8_t byte)
{
    uint8_t next = (uint8_t)((s.rx_head + 1U) & (COMM_RX_RING_SIZE - 1U));
    if (next == s.rx_tail) {
        s.err_rx_overflow = 1U;          /* đầy: bỏ byte */
        return;
    }
    s.rx_ring[s.rx_head] = byte;
    s.rx_head = next;
}

void Comm_OnUartError(uint32_t flags, bool tx_aborted)
{
    if ((flags & COMM_ERR_UART_OVERRUN) != 0UL) { s.err_overrun = 1U; }
    if ((flags & COMM_ERR_DMA) != 0UL)          { s.err_dma = 1U; }
    if ((flags & COMM_ERR_RX_OVERFLOW) != 0UL)  { s.err_rx_overflow = 1U; }
    if (tx_aborted) {
        s.tx_busy = 0U;                  /* vòng lặp chính sẽ phát lại */
    }
}
