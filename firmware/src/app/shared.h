// Data shared between tasks. Ownership rules:
//   * control task (core 1) is the only writer of ControlStatus, CalibrationData,
//     telemetry and events; it never blocks on any of them.
//   * parameters have several writers (USB/BLE console, autotune result); writes
//     are serialised by a mutex, the control task reads lock-free (seqlock).
//   * the power task is the only writer of PowerStatus.
#pragma once

#include <atomic>
#include <cstdint>

#include "balance.h"
#include "calibration.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "params.h"
#include "supervisor.h"
#include "sync.h"
#include "sysid.h"

namespace app {

// ---------------------------------------------------------------- power
struct PowerStatus {
    float v_inst = 0, v_avg = 0, i_inst = 0, i_avg = 0, power_w = 0;
    bool sensor_ok = false;
    bool present = false;      // pack connected (not just USB)
    bool above_cutoff = false; // 1 s average above bat_cut_v (with hysteresis)
    bool low_warning = false;  // 1 s average below bat_warn_v
    bool current_warning = false;
    bool overcurrent = false;  // above i_trip_a for i_trip_ms
    bool shunt_overflow = false;
    uint32_t samples = 0;
};
extern cubli::SeqLock<PowerStatus> g_power;

// ---------------------------------------------------------------- parameters
class ParamStore {
public:
    void init(const cubli::Params& p);
    cubli::Params get() const;
    uint32_t version() const { return sl_.version(); }
    bool read(cubli::Params& out) const { return sl_.read(out); }
    /// Serialised read-modify-write. fn returns false to cancel.
    template <typename Fn>
    bool update(Fn fn) {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        cubli::Params p = get();
        const bool ok = fn(p);
        if (ok) sl_.write(p);
        xSemaphoreGive(mutex_);
        return ok;
    }

private:
    cubli::SeqLock<cubli::Params> sl_;
    SemaphoreHandle_t mutex_ = nullptr;
};
extern ParamStore g_params;

// ---------------------------------------------------------------- commands to control
enum class Cmd : uint8_t {
    kArm, kDisarm, kClearFault, kBenchOn, kBenchOff,
    kBrake,          // i0: 1 = engage, 0 = release
    kMotor,          // i0: motor 0..2, f0: command -1..1
    kStop,
    kWireTest,       // i0: motor
    kEncZero,
    kCalibAxes, kCalibPoint, kCalibDeletePoint /* i0 */, kCalibClear,
    kSetCalib,       // replace calibration (from NVS reload / axis param), uses g_calib_in
    kSysIdMotor,     // i0: motor
    kAutotune,       // i0: 0 closed-loop, 1 open-loop drop test
    kAutotuneCancel,
    kLog,            // i0: duration ms
    kImuSend,        // text
    kImuRestart,
};
struct ControlCommand {
    Cmd cmd;
    int32_t i0 = 0;
    float f0 = 0;
    char text[24] = {0};
};
extern QueueHandle_t g_cmd_queue;
bool send_command(const ControlCommand& c);
bool send_command(Cmd c, int32_t i0 = 0, float f0 = 0.0f);
extern cubli::SeqLock<cubli::CalibrationData> g_calib_in;  // payload for kSetCalib

// ---------------------------------------------------------------- status from control
struct ControlStatus {
    uint32_t t_ms = 0;
    cubli::SysState state = cubli::SysState::kBoot;
    uint32_t faults = 0;
    uint32_t fault_conditions = 0;
    int8_t active_point = -1;
    int8_t nearest_point = -1;
    float nearest_deg = 0;
    char not_ready[56] = {0};
    // IMU
    uint8_t imu_state = 0;
    uint32_t imu_baud = 0;
    uint8_t imu_status = 0;
    bool imu_ok = false;
    uint32_t imu_frames_v1 = 0, imu_frames_v2 = 0, imu_crc_errors = 0, imu_gaps = 0, imu_overflows = 0;
    float imu_age_ms = 0;
    cubli::Quat q;
    cubli::Vec3 up, gyro, accel;
    float v1_roll = 0, v1_pitch = 0, v1_yaw = 0, imu_supply_v = 0;
    // motors
    int32_t enc_raw[3] = {0, 0, 0};
    int32_t enc_rel[3] = {0, 0, 0};  // since `enc zero`, inversion applied
    float wheel[3] = {0, 0, 0};
    float u[3] = {0, 0, 0};
    bool brake_released = false;
    // control
    float tilt_deg = 0;
    float theta_deg[2] = {0, 0};
    // loop timing
    float loop_hz = 0;
    uint32_t exec_us_max = 0, period_us_max = 0, period_us_min = 0, overruns = 0;
    bool deadman_tripped = false;
    // procedures
    uint8_t procedure = 0;  // Procedure enum
    float procedure_progress = 0;
};
extern cubli::SeqLock<ControlStatus> g_status;
extern cubli::SeqLock<cubli::CalibrationData> g_calib;  // current calibration (for save/show)

// ---------------------------------------------------------------- events from control
enum class Ev : uint8_t {
    kSupervisor,     // i0: SupEvent, i1: state
    kWireTestDone,   // i0 motor, f[0..2] counts: +cmd, -cmd, braked; f3 speed at +cmd
    kAxesDone,       // i0: 1 ok; f: magnitudes
    kAxesFailed,
    kPointStored,    // i0 slot, i1 kind, f0 motor
    kPointRejected,  // i0 reason
    kRecordDone,     // i0 Procedure, i1 samples
    kImuLinkChanged, // i0 state
    kCalibChanged,   // calibration should be saved by the console task
    kBenchTimeout,
};
struct ControlEvent {
    Ev ev;
    int32_t i0 = 0, i1 = 0;
    float f[6] = {0, 0, 0, 0, 0, 0};
};
extern cubli::SpscRing<ControlEvent, 32> g_events;      // -> console task
extern cubli::SpscRing<ControlEvent, 16> g_ui_events;   // -> UI task (supervisor events only)

// ---------------------------------------------------------------- recordings
enum class Procedure : uint8_t { kNone = 0, kAxes, kSysIdMotor, kAutotune, kAutotuneDrop, kLog, kWireTest };
/// Recorder buffer: owned by the control task while `busy`, by the console task
/// once kRecordDone was posted until it calls release_recording().
struct Recording {
    cubli::SysIdSample samples[1250];
    int count = 0;
    int naxes = 1;
    Procedure procedure = Procedure::kNone;
    int motor = -1;           // edge motor / sysid motor
    int point = -1;
    float kv_mean = 0;
    std::atomic<bool> busy{false};
    std::atomic<bool> ready{false};  // filled, waiting for the console task
};
extern Recording g_recording;
void release_recording();

// ---------------------------------------------------------------- telemetry
struct TelemetrySample {
    uint32_t t_ms;
    uint8_t state;
    uint8_t brake;
    uint16_t exec_us;
    float theta[2], rate[2];
    float wheel[3];
    float u[3];
    float volts, amps;
};
extern cubli::SpscRing<TelemetrySample, 128> g_telemetry;

// ---------------------------------------------------------------- misc
extern std::atomic<int64_t> g_control_heartbeat_us;
extern std::atomic<bool> g_motors_live;       // brake released by the control task
extern std::atomic<bool> g_deadman_tripped;
extern std::atomic<uint32_t> g_reset_reason;
extern std::atomic<bool> g_beep_request;      // console -> UI task

}  // namespace app
