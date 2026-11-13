#include "hal/imu_link.h"

#include <cstring>

#include "board_pins.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "hw_config.h"

namespace hal {
namespace {

constexpr uart_port_t kPort = UART_NUM_2;
constexpr uint32_t kSearchBauds[] = {hw::kImuFastBaud, hw::kImuBootBaud, 921600, 230400, 115200};
constexpr int kNumSearchBauds = sizeof(kSearchBauds) / sizeof(kSearchBauds[0]);
constexpr int64_t kSearchDwellUs = 150000;
constexpr int64_t kUpgradeTimeoutUs = 400000;
constexpr int64_t kUpgradeRetryUs = 600000;
constexpr int64_t kLinkLostUs = 300000;
constexpr uint32_t kLockFrames = 3;

}  // namespace

const char* imu_link_state_name(ImuLink::State s) {
    switch (s) {
        case ImuLink::State::kSearching: return "searching";
        case ImuLink::State::kV1: return "v1";
        case ImuLink::State::kUpgrading: return "upgrading";
        case ImuLink::State::kV2: return "v2";
    }
    return "?";
}

bool ImuLink::init() {
    uart_config_t c = {};
    c.baud_rate = static_cast<int>(kSearchBauds[0]);
    c.data_bits = UART_DATA_8_BITS;
    c.parity = UART_PARITY_DISABLE;
    c.stop_bits = UART_STOP_BITS_1;
    c.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    c.source_clk = UART_SCLK_DEFAULT;
    QueueHandle_t q = nullptr;
    if (uart_driver_install(kPort, 4096, 256, 32, &q, 0) != ESP_OK) return false;
    queue_ = q;
    uart_param_config(kPort, &c);
    uart_set_pin(kPort, board::kPinImuTx, board::kPinImuRx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Wake the control task ~3 character times after a frame ends.
    uart_set_rx_timeout(kPort, 3);
    uart_set_rx_full_threshold(kPort, 64);
    baud_ = kSearchBauds[0];
    baud_index_ = 0;
    state_ = State::kSearching;
    return true;
}

void ImuLink::set_baud(uint32_t baud) {
    uart_set_baudrate(kPort, baud);
    uart_flush_input(kPort);
    baud_ = baud;
}

void ImuLink::write_text(const char* text) {
    uart_write_bytes(kPort, text, std::strlen(text));
    uart_write_bytes(kPort, "\r\n", 2);
}

bool ImuLink::send_command(const char* text) {
    if (cmd_pending_) return false;
    std::strncpy(pending_cmd_, text, sizeof(pending_cmd_) - 1);
    pending_cmd_[sizeof(pending_cmd_) - 1] = '\0';
    cmd_pending_ = true;
    return true;
}

void ImuLink::restart_search() {
    state_ = State::kSearching;
    baud_index_ = 0;
    set_baud(kSearchBauds[0]);
    frames_in_state_ = 0;
    state_since_us_ = 0;
}

bool ImuLink::poll(uint32_t timeout_us, int64_t now_us) {
    frames_in_poll_ = 0;
    bool got_v2 = false;
    uart_event_t ev;
    TickType_t ticks = pdMS_TO_TICKS((timeout_us + 999) / 1000);
    if (ticks == 0) ticks = 1;
    if (xQueueReceive(static_cast<QueueHandle_t>(queue_), &ev, ticks) == pdTRUE) {
        if (ev.type == UART_FIFO_OVF || ev.type == UART_BUFFER_FULL) {
            ++overflows_;
            uart_flush_input(kPort);
            xQueueReset(static_cast<QueueHandle_t>(queue_));
        }
    }
    // Drain everything buffered (events only wake us up).
    uint8_t buf[256];
    for (int guard = 0; guard < 16; ++guard) {
        size_t avail = 0;
        uart_get_buffered_data_len(kPort, &avail);
        if (avail == 0) break;
        const int n = uart_read_bytes(kPort, buf, avail < sizeof buf ? avail : sizeof buf, 0);
        if (n <= 0) break;
        for (int i = 0; i < n; ++i) {
            const auto r = parser_.feed(buf[i]);
            if (r == cubli::ImuFrameParser::Result::kFrameV2) {
                latest_ = parser_.v2();
                last_v2_us_ = last_frame_us_ = now_us;
                got_v2 = true;
                ++frames_in_poll_;
                if (state_ == State::kSearching || state_ == State::kUpgrading) ++frames_in_state_;
                if (state_ != State::kV2 && frames_in_state_ >= kLockFrames) {
                    state_ = State::kV2;
                    state_since_us_ = now_us;
                }
            } else if (r == cubli::ImuFrameParser::Result::kFrameV1) {
                last_frame_us_ = now_us;
                on_frame_v1(now_us);
            }
        }
    }
    while (xQueueReceive(static_cast<QueueHandle_t>(queue_), &ev, 0) == pdTRUE) {
    }

    if (state_since_us_ == 0) state_since_us_ = now_us;
    switch (state_) {
        case State::kSearching:
            if (now_us - state_since_us_ > kSearchDwellUs) {
                baud_index_ = (baud_index_ + 1) % kNumSearchBauds;
                set_baud(kSearchBauds[baud_index_]);
                state_since_us_ = now_us;
                frames_in_state_ = 0;
            }
            break;
        case State::kV1:
            if (now_us - last_frame_us_ > kLinkLostUs) {
                restart_search();
            } else if (now_us - last_upgrade_us_ > kUpgradeRetryUs) {
                // Ask the IMU PCB for frame v2; it switches to 460800 after the
                // current frame plus 20 ms of silence.
                write_text("CUBLI");
                uart_wait_tx_done(kPort, pdMS_TO_TICKS(20));
                last_upgrade_us_ = now_us;
                set_baud(hw::kImuFastBaud);
                state_ = State::kUpgrading;
                state_since_us_ = now_us;
                frames_in_state_ = 0;
            }
            break;
        case State::kUpgrading:
            if (now_us - state_since_us_ > kUpgradeTimeoutUs) {
                // No v2: IMU PCB firmware without the CubliV2 patch. Stay on v1.
                set_baud(hw::kImuBootBaud);
                state_ = State::kSearching;
                baud_index_ = 1;
                state_since_us_ = now_us;
                frames_in_state_ = 0;
            }
            break;
        case State::kV2:
            if (now_us - last_v2_us_ > kLinkLostUs) restart_search();
            break;
    }

    if (cmd_pending_ && (state_ == State::kV1 || state_ == State::kV2)) {
        write_text(pending_cmd_);
        cmd_pending_ = false;
    }
    return got_v2;
}

void ImuLink::on_frame_v1(int64_t now_us) {
    if (state_ == State::kSearching) {
        if (++frames_in_state_ >= kLockFrames) {
            state_ = State::kV1;
            state_since_us_ = now_us;
            last_upgrade_us_ = now_us - kUpgradeRetryUs + 50000;  // upgrade after 50 ms
        }
    }
}

}  // namespace hal
