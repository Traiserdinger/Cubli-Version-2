#include "hal/motor_output.h"

#include <cmath>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "hw_config.h"

namespace hal {
namespace {

constexpr ledc_mode_t kMode = LEDC_HIGH_SPEED_MODE;
constexpr ledc_timer_t kTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kChannel[board::kNumMotors] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};

void gpio_out(int pin, int level) {
    gpio_set_level(static_cast<gpio_num_t>(pin), level);
    gpio_config_t c = {};
    c.pin_bit_mask = 1ULL << pin;
    c.mode = GPIO_MODE_OUTPUT;
    c.pull_up_en = GPIO_PULLUP_DISABLE;
    c.pull_down_en = GPIO_PULLDOWN_DISABLE;
    c.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&c);
    gpio_set_level(static_cast<gpio_num_t>(pin), level);
}

}  // namespace

void MotorOutput::init() {
    // 1. Brake all motors (it already is, through R32; make it explicit).
    gpio_out(board::kPinBrake, 0);
    brake_released_ = false;
    // 2. PWM lines HIGH = stop, as plain GPIOs, before LEDC exists.
    for (int k = 0; k < board::kNumMotors; ++k) {
        gpio_out(board::kPinPwm[k], 1);
        gpio_out(board::kPinDir[k], 0);
    }
    // 3. LEDC at 100 % duty (constant HIGH).
    ledc_timer_config_t t = {};
    t.speed_mode = kMode;
    t.duty_resolution = static_cast<ledc_timer_bit_t>(hw::kPwmResolutionBits);
    t.timer_num = kTimer;
    t.freq_hz = hw::kPwmFrequencyHz;
    t.clk_cfg = LEDC_USE_APB_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    for (int k = 0; k < board::kNumMotors; ++k) {
        ledc_channel_config_t c = {};
        c.gpio_num = board::kPinPwm[k];
        c.speed_mode = kMode;
        c.channel = kChannel[k];
        c.intr_type = LEDC_INTR_DISABLE;
        c.timer_sel = kTimer;
        c.duty = hw::kPwmDutyMax;
        c.hpoint = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&c));
        duty_[k] = hw::kPwmDutyMax;
        command_[k] = 0.0f;
    }
    ready_ = true;
}

void MotorOutput::set(int motor, float u, bool dir_invert) {
    if (!ready_ || motor < 0 || motor >= board::kNumMotors) return;
    if (!std::isfinite(u)) u = 0.0f;
    if (u > 1.0f) u = 1.0f;
    if (u < -1.0f) u = -1.0f;
    command_[motor] = u;
    if (u != 0.0f) {
        int level = u > 0.0f ? 1 : 0;
        if (dir_invert) level ^= 1;
        if (level != dir_level_[motor]) {
            gpio_set_level(static_cast<gpio_num_t>(board::kPinDir[motor]), level);
            dir_level_[motor] = level;
        }
    }
    const float mag = std::fabs(u);
    const uint32_t duty = hw::kPwmDutyMax - static_cast<uint32_t>(std::lround(mag * static_cast<float>(hw::kPwmDutyMax)));
    if (duty != duty_[motor]) {
        ledc_set_duty(kMode, kChannel[motor], duty);
        ledc_update_duty(kMode, kChannel[motor]);
        duty_[motor] = duty;
    }
}

void MotorOutput::stop_all() {
    for (int k = 0; k < board::kNumMotors; ++k) set(k, 0.0f, false);
}

void MotorOutput::set_brake_released(bool released) {
    // Releasing is only allowed while every output is at the stop level, so a
    // stale command can never start a motor at the moment the brake opens
    // (context.md §5.3 rule 4). The caller then ramps commands from zero.
    if (released == brake_released_) return;
    if (released) {
        for (int k = 0; k < board::kNumMotors; ++k)
            if (duty_[k] != hw::kPwmDutyMax) return;
    }
    gpio_set_level(static_cast<gpio_num_t>(board::kPinBrake), released ? 1 : 0);
    brake_released_ = released;
}

}  // namespace hal
