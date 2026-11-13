// Motor outputs: LEDC PWM (inverted: HIGH = stop), DIR and the shared BRAKE line.
//
// Safety (context.md §5.3): at reset the pull-downs give BRAKE = LOW (braked)
// and PWM = LOW (= full-speed command). init() therefore drives BRAKE LOW and the
// PWM pins HIGH as plain GPIOs first, then hands the PWM pins to LEDC at 100 %
// duty (constant HIGH = stop). BRAKE is released only by set_brake_released(true).
#pragma once

#include <cstdint>

#include "board_pins.h"

namespace hal {

class MotorOutput {
public:
    /// Call first in app_main, before anything else touches the motor pins.
    void init();

    /// u in [-1, 1]; sign selects DIR (optionally inverted), |u| is the speed
    /// command. u == 0 gives duty = max (output constantly HIGH = stop).
    void set(int motor, float u, bool dir_invert);
    void stop_all();

    /// true -> BRAKE HIGH (run enabled), false -> BRAKE LOW (brake all motors).
    void set_brake_released(bool released);
    bool brake_released() const { return brake_released_; }

    /// Last applied direction level (for the reversal rule) and command.
    int dir_level(int motor) const { return dir_level_[motor]; }
    float command(int motor) const { return command_[motor]; }

private:
    bool ready_ = false;
    bool brake_released_ = false;
    int dir_level_[board::kNumMotors] = {0, 0, 0};
    float command_[board::kNumMotors] = {0, 0, 0};
    uint32_t duty_[board::kNumMotors] = {0, 0, 0};
};

}  // namespace hal
