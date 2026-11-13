#include "supervisor.h"

#include <cstdio>

namespace cubli {

const char* sys_state_name(SysState s) {
    switch (s) {
        case SysState::kBoot: return "BOOT";
        case SysState::kIdle: return "IDLE";
        case SysState::kCalibrate: return "CALIBRATE";
        case SysState::kReady: return "READY";
        case SysState::kArming: return "ARMING";
        case SysState::kArmedEdge: return "ARMED_EDGE";
        case SysState::kArmedVertex: return "ARMED_VERTEX";
        case SysState::kBench: return "BENCH";
        case SysState::kFault: return "FAULT";
    }
    return "?";
}

const char* fault_names(uint32_t faults, char* buf, int len) {
    static const char* const kNames[] = {"battery_low", "over_current", "imu_lost", "wheel_overspeed",
                                         "deadman", "overrun", "battery_missing", "numeric"};
    int pos = 0;
    if (len > 0) buf[0] = '\0';
    for (int i = 0; i < 8 && pos < len; ++i) {
        if (faults & (1u << i)) pos += std::snprintf(buf + pos, len - pos, "%s%s", pos ? " " : "", kNames[i]);
    }
    if (pos == 0 && len > 0) std::snprintf(buf, len, "none");
    return buf;
}

bool Supervisor::ready_ok(const SupervisorInputs& in) const {
    return in.imu_ok && in.battery_present && in.battery_ok && in.axes_valid && in.point_count > 0;
}

const char* Supervisor::not_ready_reason(const SupervisorInputs& in) const {
    if (faults_) return "fault latched (fault clear)";
    if (!in.battery_present) return "no battery (USB power only)";
    if (!in.battery_ok) return "battery below cut-off";
    if (!in.imu_ok) return "IMU PCB not streaming frame v2 / EKF not ready";
    if (!in.axes_valid) return "motor axes not calibrated (calib axes)";
    if (in.point_count == 0) return "no balance point stored (calib point)";
    return "";
}

SupEvent Supervisor::update(const SupervisorInputs& in, SupRequest req, const SupervisorConfig& cfg) {
    SupEvent ev = SupEvent::kNone;
    const SysState prev = state_;
    t_state_ += in.dt;

    // ---- fault latching ----
    uint32_t latch = in.fault_conditions & fault::kAlwaysLatched;
    if (motors_allowed()) latch |= in.fault_conditions;
    if (motors_allowed() && !in.battery_present) latch |= fault::kBatteryMissing;
    if (latch & ~faults_) ev = SupEvent::kFault;
    faults_ |= latch;
    if (faults_ && state_ != SysState::kFault) {
        state_ = SysState::kFault;
        active_point_ = -1;
    }

    switch (state_) {
        case SysState::kBoot:
            if (t_state_ >= cfg.boot_time_s) state_ = SysState::kIdle;
            break;

        case SysState::kIdle:
        case SysState::kReady: {
            const SysState next = ready_ok(in) ? SysState::kReady : SysState::kIdle;
            if (next != state_) ev = next == SysState::kReady ? SupEvent::kReady : SupEvent::kNotReady;
            state_ = next;
            if (in.calibrating) {
                state_ = SysState::kCalibrate;
            } else if (req == SupRequest::kBenchOn) {
                state_ = SysState::kBench;
            } else if (req == SupRequest::kArm && state_ == SysState::kReady) {
                state_ = SysState::kArming;
                t_hold_ = 0.0f;
                ev = SupEvent::kArming;
            }
            break;
        }

        case SysState::kCalibrate:
            if (!in.calibrating) state_ = SysState::kIdle;
            break;

        case SysState::kBench:
            if (req == SupRequest::kBenchOff || req == SupRequest::kDisarm) state_ = SysState::kIdle;
            if (in.calibrating) state_ = SysState::kCalibrate;
            break;

        case SysState::kArming: {
            if (req == SupRequest::kDisarm) {
                state_ = SysState::kReady;
                ev = SupEvent::kDisarmed;
                break;
            }
            if (!ready_ok(in)) {
                state_ = SysState::kIdle;
                ev = SupEvent::kNotReady;
                break;
            }
            if (t_state_ >= cfg.arm_timeout_s) {
                state_ = SysState::kReady;
                ev = SupEvent::kArmTimeout;
                break;
            }
            const bool near = in.nearest_point >= 0 && in.nearest_angle < cfg.arm_capture_rad &&
                              in.body_rate < cfg.arm_rate_rads &&
                              (in.nearest_is_edge || in.nearest_is_vertex);
            t_hold_ = near ? t_hold_ + in.dt : 0.0f;
            if (near && t_hold_ >= cfg.arm_hold_s) {
                active_point_ = in.nearest_point;
                state_ = in.nearest_is_vertex ? SysState::kArmedVertex : SysState::kArmedEdge;
                ev = SupEvent::kArmed;
            }
            break;
        }

        case SysState::kArmedEdge:
        case SysState::kArmedVertex:
            if (req == SupRequest::kDisarm) {
                state_ = SysState::kReady;
                active_point_ = -1;
                ev = SupEvent::kDisarmed;
            } else if (in.tilt_rad > cfg.tilt_limit_rad) {
                active_point_ = -1;
                state_ = cfg.rearm ? SysState::kArming : SysState::kReady;
                t_hold_ = 0.0f;
                ev = SupEvent::kFell;
            } else if (!in.imu_ok) {
                // Normally caught as fault::kImuLost first; belt and braces.
                faults_ |= fault::kImuLost;
                state_ = SysState::kFault;
                active_point_ = -1;
                ev = SupEvent::kFault;
            }
            break;

        case SysState::kFault:
            if (req == SupRequest::kClearFault && (in.fault_conditions & faults_) == 0) {
                faults_ = 0;
                state_ = SysState::kIdle;
                ev = SupEvent::kFaultCleared;
            }
            break;
    }
    if (state_ != prev) t_state_ = 0.0f;
    return ev;
}

}  // namespace cubli
