// Power monitor task (core 0, 50 Hz): INA219 -> filtered pack voltage/current,
// battery thresholds (context.md §3) and over-current detection.
#include "app/power_task.h"

#include <cmath>

#include "app/shared.h"
#include "filters.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/ina219_dev.h"
#include "hw_config.h"

namespace app {
namespace {

constexpr int kRateHz = 50;
constexpr float kCutoffHysteresisV = 0.2f;

void power_task(void* arg) {
    auto* ina = static_cast<hal::Ina219*>(arg);
    cubli::MovingAverage<kRateHz> v_avg, i_avg;  // 1 s
    PowerStatus st;
    float cfg_shunt = -1.0f, cfg_range = -1.0f;
    float over_ms = 0.0f;
    bool above = false;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / kRateHz));
        const cubli::Params p = g_params.get();
        const float shunt = p.shunt_mohm * 1e-3f;
        if (shunt != cfg_shunt || p.i_range_a != cfg_range || !ina->present()) {
            if (ina->configure(shunt, p.i_range_a)) {
                cfg_shunt = shunt;
                cfg_range = p.i_range_a;
            } else {
                cfg_shunt = -1.0f;  // retry next time
            }
        }
        float v = 0, i = 0;
        bool ovf = false;
        st.sensor_ok = cfg_shunt > 0.0f && ina->read(&v, &i, &ovf);
        if (!st.sensor_ok) {
            st.present = st.above_cutoff = false;
            g_power.write(st);
            continue;
        }
        v *= p.vbat_scale;
        i -= p.i_offset_a;
        st.v_inst = v;
        st.i_inst = i;
        st.power_w = v * i;
        st.shunt_overflow = ovf;
        st.v_avg = v_avg.update(v);
        st.i_avg = i_avg.update(i);
        st.present = v > hw::kBatteryPresentMinV;
        if (!st.present) {
            v_avg.reset();
            above = false;
        } else if (v_avg.full()) {
            // Hysteresis: recover only 0.2 V above the cut-off.
            above = above ? st.v_avg >= p.bat_cut_v : st.v_avg >= p.bat_cut_v + kCutoffHysteresisV;
        } else {
            above = false;  // wait for a full 1 s average after connecting
        }
        st.above_cutoff = above;
        st.low_warning = st.present && v_avg.full() && st.v_avg < p.bat_warn_v;
        st.current_warning = std::fabs(i) > p.i_warn_a;
        over_ms = (std::fabs(i) > p.i_trip_a || ovf) ? over_ms + 1000.0f / kRateHz : 0.0f;
        st.overcurrent = over_ms >= p.i_trip_ms;
        ++st.samples;
        g_power.write(st);
    }
}

}  // namespace

void start_power_task(hal::Ina219* ina) {
    xTaskCreatePinnedToCore(power_task, "power", 4096, ina, 5, nullptr, 0);
}

}  // namespace app
