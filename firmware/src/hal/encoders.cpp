#include "hal/encoders.h"

#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "hw_config.h"

namespace hal {
namespace {
constexpr const char* kTag = "enc";
constexpr int kHigh = 30000;
constexpr int kLow = -30000;
}  // namespace

bool Encoders::init() {
    ok_ = true;
    for (int k = 0; k < board::kNumMotors; ++k) {
        pcnt_unit_config_t uc = {};
        uc.high_limit = kHigh;
        uc.low_limit = kLow;
        uc.flags.accum_count = 1;
        pcnt_unit_handle_t unit = nullptr;
        if (pcnt_new_unit(&uc, &unit) != ESP_OK) {
            ESP_LOGE(kTag, "unit %d", k);
            ok_ = false;
            continue;
        }
        pcnt_glitch_filter_config_t fc = {};
        fc.max_glitch_ns = hw::kEncoderGlitchNs;
        pcnt_unit_set_glitch_filter(unit, &fc);

        pcnt_chan_config_t ca = {};
        ca.edge_gpio_num = board::kPinEncA[k];
        ca.level_gpio_num = board::kPinEncB[k];
        pcnt_channel_handle_t cha = nullptr;
        pcnt_new_channel(unit, &ca, &cha);
        pcnt_chan_config_t cb = {};
        cb.edge_gpio_num = board::kPinEncB[k];
        cb.level_gpio_num = board::kPinEncA[k];
        pcnt_channel_handle_t chb = nullptr;
        pcnt_new_channel(unit, &cb, &chb);

        // Standard x4 quadrature table (ESP-IDF rotary encoder example).
        pcnt_channel_set_edge_action(cha, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
        pcnt_channel_set_level_action(cha, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        pcnt_channel_set_edge_action(chb, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        pcnt_channel_set_level_action(chb, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

        pcnt_unit_add_watch_point(unit, kHigh);
        pcnt_unit_add_watch_point(unit, kLow);
        pcnt_unit_enable(unit);
        pcnt_unit_clear_count(unit);
        pcnt_unit_start(unit);
        unit_[k] = unit;
    }
    return ok_;
}

int32_t Encoders::count(int motor) const {
    if (motor < 0 || motor >= board::kNumMotors || !unit_[motor]) return 0;
    int v = 0;
    pcnt_unit_get_count(static_cast<pcnt_unit_handle_t>(unit_[motor]), &v);
    return v;
}

}  // namespace hal
