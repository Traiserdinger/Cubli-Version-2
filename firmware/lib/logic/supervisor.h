// Supervisor state machine (hardware independent).
//
//   BOOT -> IDLE <-> READY -> ARMING -> ARMED_EDGE / ARMED_VERTEX
//            |  ^      |        ^  |            |
//            v  |      v        |  +-- timeout  +-- tilt > limit (fell) -> READY
//         CALIBRATE  BENCH      +------------------ rearm = 1 ----------/
//   any state -- latched fault --> FAULT -- "fault clear", cause gone --> IDLE
//
// READY means: IMU healthy (frame v2, EKF ready), battery present and above
// cut-off, motor axes calibrated and at least one balance point stored.
// Motors only run in ARMED_*, BENCH and CALIBRATE (axis/sysid procedures).
#pragma once

#include <cstdint>

namespace cubli {

enum class SysState : uint8_t {
    kBoot = 0,
    kIdle,
    kCalibrate,
    kReady,
    kArming,
    kArmedEdge,
    kArmedVertex,
    kBench,
    kFault,
};
const char* sys_state_name(SysState s);

namespace fault {
inline constexpr uint32_t kBatteryLow = 1u << 0;     // 1 s average below cut-off
inline constexpr uint32_t kOverCurrent = 1u << 1;    // above trip for trip time
inline constexpr uint32_t kImuLost = 1u << 2;        // no frame within imu_tout while motors run
inline constexpr uint32_t kWheelOverspeed = 1u << 3;
inline constexpr uint32_t kDeadman = 1u << 4;        // control loop stalled (timer brake fired)
inline constexpr uint32_t kOverrun = 1u << 5;        // repeated control-loop overruns
inline constexpr uint32_t kBatteryMissing = 1u << 6; // pack vanished while motors ran
inline constexpr uint32_t kNumeric = 1u << 7;        // NaN/Inf in the controller
// Always latched (in every state); the others only while motors may run.
inline constexpr uint32_t kAlwaysLatched = kOverCurrent | kWheelOverspeed | kDeadman | kOverrun | kNumeric;
}  // namespace fault
/// Writes a space-separated list of fault names; returns buf.
const char* fault_names(uint32_t faults, char* buf, int len);

enum class SupRequest : uint8_t { kNone, kArm, kDisarm, kClearFault, kBenchOn, kBenchOff };

enum class SupEvent : uint8_t { kNone, kReady, kNotReady, kArming, kArmed, kDisarmed, kFell, kArmTimeout, kFault, kFaultCleared };

struct SupervisorConfig {
    float boot_time_s = 0.5f;
    float arm_capture_rad = 0.035f;
    float arm_rate_rads = 0.3f;
    float arm_hold_s = 0.3f;
    float arm_timeout_s = 30.0f;
    float tilt_limit_rad = 0.17f;
    bool rearm = false;
};

struct SupervisorInputs {
    float dt = 0.001f;
    bool imu_ok = false;
    bool battery_present = false;
    bool battery_ok = false;
    uint32_t fault_conditions = 0;  // fault:: bits currently true
    bool axes_valid = false;
    int point_count = 0;
    bool calibrating = false;       // a calibration procedure is running
    int nearest_point = -1;         // already filtered by the mode parameter
    float nearest_angle = 1e9f;
    bool nearest_is_vertex = false;
    bool nearest_is_edge = false;
    float body_rate = 0.0f;
    float tilt_rad = 0.0f;          // tilt from the active point while armed
};

class Supervisor {
public:
    /// One tick. Returns an event for the UI (sounds, messages).
    SupEvent update(const SupervisorInputs& in, SupRequest req, const SupervisorConfig& cfg);

    SysState state() const { return state_; }
    uint32_t faults() const { return faults_; }
    int active_point() const { return active_point_; }
    bool armed() const { return state_ == SysState::kArmedEdge || state_ == SysState::kArmedVertex; }
    bool motors_allowed() const {
        return armed() || state_ == SysState::kBench || state_ == SysState::kCalibrate;
    }
    /// Why the cube is not READY (for `status`).
    const char* not_ready_reason(const SupervisorInputs& in) const;

private:
    bool ready_ok(const SupervisorInputs& in) const;

    SysState state_ = SysState::kBoot;
    uint32_t faults_ = 0;
    int active_point_ = -1;
    float t_state_ = 0.0f, t_hold_ = 0.0f;
};

}  // namespace cubli
