// UART2 link to the IMU PCB: baud search, upgrade from frame v1 to frame v2
// ("CUBLI" command, 460800 baud), frame parsing and link statistics.
// Runs entirely inside the control task (no extra task, no locks).
#pragma once

#include <cstdint>

#include "imu_frame.h"

namespace hal {

class ImuLink {
public:
    enum class State : uint8_t { kSearching, kV1, kUpgrading, kV2 };

    bool init();
    /// Waits up to timeout_us for UART data, parses it and runs the link state
    /// machine. Returns true if at least one new v2 frame arrived; the newest is
    /// in latest(). now_us must be monotonic.
    bool poll(uint32_t timeout_us, int64_t now_us);

    const cubli::ImuFrameV2& latest() const { return latest_; }
    const cubli::ImuFrameV1& latest_v1() const { return parser_.v1(); }
    int64_t last_v2_us() const { return last_v2_us_; }
    int64_t last_frame_us() const { return last_frame_us_; }
    State state() const { return state_; }
    uint32_t baud() const { return baud_; }
    const cubli::ImuFrameParser& parser() const { return parser_; }
    uint32_t frames_in_poll() const { return frames_in_poll_; }
    uint32_t uart_overflows() const { return overflows_; }

    /// Queue an ASCII command for the IMU PCB (sent with CR LF from poll()).
    bool send_command(const char* text);
    /// Ask for frame v2 again (e.g. after the IMU PCB was power-cycled).
    void restart_search();

private:
    void set_baud(uint32_t baud);
    void write_text(const char* text);
    void on_frame_v1(int64_t now_us);

    void* queue_ = nullptr;
    cubli::ImuFrameParser parser_;
    cubli::ImuFrameV2 latest_;
    State state_ = State::kSearching;
    uint32_t baud_ = 0;
    int baud_index_ = 0;
    int64_t state_since_us_ = 0, last_v2_us_ = 0, last_frame_us_ = 0, last_upgrade_us_ = 0;
    uint32_t frames_in_state_ = 0, frames_in_poll_ = 0, overflows_ = 0;
    char pending_cmd_[24] = {0};
    bool cmd_pending_ = false;
};

const char* imu_link_state_name(ImuLink::State s);

}  // namespace hal
