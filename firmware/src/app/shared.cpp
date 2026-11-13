#include "app/shared.h"

#include <cstring>

namespace app {

cubli::SeqLock<PowerStatus> g_power;
ParamStore g_params;
QueueHandle_t g_cmd_queue = nullptr;
cubli::SeqLock<cubli::CalibrationData> g_calib_in;
cubli::SeqLock<ControlStatus> g_status;
cubli::SeqLock<cubli::CalibrationData> g_calib;
cubli::SpscRing<ControlEvent, 32> g_events;
cubli::SpscRing<ControlEvent, 16> g_ui_events;
Recording g_recording;
cubli::SpscRing<TelemetrySample, 128> g_telemetry;
std::atomic<int64_t> g_control_heartbeat_us{0};
std::atomic<bool> g_motors_live{false};
std::atomic<bool> g_deadman_tripped{false};
std::atomic<uint32_t> g_reset_reason{0};
std::atomic<bool> g_beep_request{false};

void ParamStore::init(const cubli::Params& p) {
    if (!mutex_) mutex_ = xSemaphoreCreateMutex();
    sl_.write(p);
}

cubli::Params ParamStore::get() const {
    cubli::Params p;
    while (!sl_.read(p)) {
    }
    return p;
}

bool send_command(const ControlCommand& c) {
    return g_cmd_queue && xQueueSend(g_cmd_queue, &c, pdMS_TO_TICKS(20)) == pdTRUE;
}

bool send_command(Cmd c, int32_t i0, float f0) {
    ControlCommand cc;
    cc.cmd = c;
    cc.i0 = i0;
    cc.f0 = f0;
    return send_command(cc);
}

void release_recording() {
    g_recording.ready.store(false);
    g_recording.busy.store(false);
}

}  // namespace app
