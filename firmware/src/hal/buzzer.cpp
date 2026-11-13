#include "hal/buzzer.h"

#include "board_pins.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "hw_config.h"

namespace hal {
namespace {
constexpr ledc_mode_t kMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kTimer = LEDC_TIMER_1;
constexpr ledc_channel_t kChannel = LEDC_CHANNEL_3;
constexpr ledc_timer_bit_t kBits = LEDC_TIMER_10_BIT;
}  // namespace

void Buzzer::init() {
    ledc_timer_config_t t = {};
    t.speed_mode = kMode;
    t.duty_resolution = kBits;
    t.timer_num = kTimer;
    t.freq_hz = hw::kBuzzerDefaultHz;
    t.clk_cfg = LEDC_AUTO_CLK;
    if (ledc_timer_config(&t) != ESP_OK) return;
    ledc_channel_config_t c = {};
    c.gpio_num = board::kPinBuzzer;
    c.speed_mode = kMode;
    c.channel = kChannel;
    c.timer_sel = kTimer;
    c.duty = 0;  // LOW = transistor off = silent
    ready_ = ledc_channel_config(&c) == ESP_OK;
}

void Buzzer::tone(uint32_t hz) {
    if (!ready_ || hz == hz_) return;
    if (hz == 0) {
        ledc_set_duty(kMode, kChannel, 0);
    } else {
        ledc_set_freq(kMode, kTimer, hz);
        ledc_set_duty(kMode, kChannel, 512);
    }
    ledc_update_duty(kMode, kChannel);
    hz_ = hz;
}

void StatusLed::init() {
    gpio_config_t c = {};
    c.pin_bit_mask = 1ULL << board::kPinStatusLed;
    c.mode = GPIO_MODE_OUTPUT;
    gpio_config(&c);
    set(false);
}

void StatusLed::set(bool on) { gpio_set_level(static_cast<gpio_num_t>(board::kPinStatusLed), on ? 1 : 0); }

void BootButton::init() {
    gpio_config_t c = {};
    c.pin_bit_mask = 1ULL << board::kPinBootButton;
    c.mode = GPIO_MODE_INPUT;
    c.pull_up_en = GPIO_PULLUP_DISABLE;  // R4 10k external pull-up
    gpio_config(&c);
}

bool BootButton::pressed() const { return gpio_get_level(static_cast<gpio_num_t>(board::kPinBootButton)) == 0; }

}  // namespace hal
