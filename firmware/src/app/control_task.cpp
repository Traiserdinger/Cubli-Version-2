#include "app/control_task.h"

#include <cmath>
#include <cstring>

#include "app/shared.h"
#include "balance.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/encoders.h"
#include "hal/imu_link.h"
#include "hal/motor_output.h"
#include "hw_config.h"
#include "imu_frame.h"
#include "supervisor.h"
#include "wheel_speed.h"

namespace app {
namespace {

using cubli::BalanceMode;
using cubli::PointKind;
using cubli::SupEvent;
using cubli::SupRequest;
using cubli::SysState;
using cubli::Vec3;

constexpr uint32_t kIdlePollUs = 2000;     // run at least every 2 ms without frames
constexpr int64_t kImuReadyMaxAgeUs = 100000;
constexpr int64_t kDeadmanUs = 20000;
constexpr float kBenchHoldS = 3.0f;
constexpr float kOverrunExecUs = 900.0f;

void deadman_cb(void*) {
    // esp_timer task (core 0). If the control task stops updating its heartbeat
    // while the brake is released, brake directly at GPIO level.
    if (!g_motors_live.load()) return;
    const int64_t age = esp_timer_get_time() - g_control_heartbeat_us.load();
    if (age > kDeadmanUs) {
        gpio_set_level(static_cast<gpio_num_t>(board::kPinBrake), 0);
        g_deadman_tripped.store(true);
    }
}

void post(Ev ev, int32_t i0 = 0, int32_t i1 = 0, float f0 = 0, float f1 = 0, float f2 = 0, float f3 = 0,
          float f4 = 0, float f5 = 0) {
    ControlEvent e;
    e.ev = ev;
    e.i0 = i0;
    e.i1 = i1;
    e.f[0] = f0; e.f[1] = f1; e.f[2] = f2; e.f[3] = f3; e.f[4] = f4; e.f[5] = f5;
    g_events.push(e);
}

class Prbs {
public:
    float next(float amp) {
        const uint16_t bit = static_cast<uint16_t>(((s_ >> 0) ^ (s_ >> 2) ^ (s_ >> 3) ^ (s_ >> 5)) & 1u);
        s_ = static_cast<uint16_t>((s_ >> 1) | (bit << 15));
        return (s_ & 1u) ? amp : -amp;
    }
    explicit Prbs(uint16_t seed) : s_(seed) {}

private:
    uint16_t s_;
};

class ControlLoop {
public:
    ControlLoop(hal::MotorOutput* m, hal::Encoders* e, hal::ImuLink* i, const cubli::CalibrationData& c)
        : motors_(m), enc_(e), imu_(i), cal_(c) {}

    void run();

private:
    void refresh_params();
    void handle(const ControlCommand& c);
    void step(int64_t now_us, bool new_frame);
    void compute_outputs(int64_t now_us, float dt, float u[3], bool& want_release);
    void apply_outputs(const float u_in[3], bool want_release, float dt);
    void enter_armed(SysState s);
    void abort_procedure();
    void finish_recording();
    void record_balance(const cubli::BalanceOutput& out, const float u[3]);
    void publish_status(int64_t now_us);
    void publish_calib() { g_calib.write(cal_); }
    float max_wheel_speed() const {
        float m = 0;
        for (float w : wheel_speed_) m = std::fmax(m, std::fabs(w));
        return m;
    }
    bool calibrating() const {
        return procedure_ == Procedure::kAxes || procedure_ == Procedure::kSysIdMotor;
    }

    hal::MotorOutput* motors_;
    hal::Encoders* enc_;
    hal::ImuLink* imu_;
    cubli::CalibrationData cal_;
    cubli::MotorAxes axes_;
    cubli::Params p_{};
    uint32_t p_version_ = 0xFFFFFFFF;
    cubli::BalanceConfig bcfg_;
    cubli::SupervisorConfig scfg_;
    cubli::Supervisor sup_;
    cubli::SupervisorInputs sin_;
    cubli::BalanceController bal_;
    cubli::BalanceOutput last_out_;
    SupRequest req_ = SupRequest::kNone;

    // sensors
    cubli::WheelSpeedEstimator wheel_[3];
    float wheel_speed_[3] = {0, 0, 0};
    int32_t enc_count_[3] = {0, 0, 0};
    int32_t enc_zero_[3] = {0, 0, 0};
    Vec3 up_{0, 0, 1}, gyro_, accel_;
    uint8_t imu_status_ = 0;
    bool imu_ok_ = false;
    int64_t imu_age_us_ = 1000000000;
    hal::ImuLink::State last_link_state_ = hal::ImuLink::State::kSearching;
    PowerStatus power_;

    // timing
    int64_t last_us_ = 0, last_status_us_ = 0, last_tlm_us_ = 0, stats_window_us_ = 0;
    uint32_t steps_in_window_ = 0, exec_max_ = 0, period_max_ = 0, period_min_ = 0xFFFFFFFF, overruns_ = 0;
    float overrun_leak_ = 0, loop_hz_ = 0;
    uint32_t pub_exec_max_ = 0, pub_period_max_ = 0, pub_period_min_ = 0;
    uint32_t last_exec_us_ = 0;
    bool numeric_fault_ = false;

    // stop strategy
    int64_t inactive_since_us_ = 0;
    bool was_active_ = false;

    // bench
    float bench_u_[3] = {0, 0, 0};
    int64_t bench_deadline_us_ = 0;
    bool bench_brake_engaged_ = false;
    bool bench_timeout_posted_ = true;

    // procedures
    Procedure procedure_ = Procedure::kNone;
    int64_t proc_start_us_ = 0;
    int proc_motor_ = 0;
    int proc_phase_ = 0;
    int64_t phase_start_us_ = 0;
    int32_t phase_count_[6] = {0, 0, 0, 0, 0, 0};
    float wiretest_speed_ = 0;
    bool brake_test_ = false;  // wiretest last phase: PWM commanded while BRAKE is LOW
    cubli::AxisCalibrator axis_cal_;
    bool drop_pending_ = false;
    int rec_div_ = 0;
    int64_t last_rec_us_ = 0;
    float rec_target_s_ = 0;
    Prbs prbs0_{0xACE1u}, prbs1_{0x1D87u};
    float prbs_v_[2] = {0, 0};
    int prbs_div_ = 0;
    // point capture
    bool point_capture_ = false;
    int64_t point_start_us_ = 0;
    Vec3 point_sum_;
    int point_n_ = 0;
    float point_rate_max_ = 0;
};

void ControlLoop::refresh_params() {
    const uint32_t v = g_params.version();
    if (v == p_version_) return;
    cubli::Params np;
    if (!g_params.read(np)) return;
    p_ = np;
    p_version_ = v;
    bcfg_.edge = {{p_.e_k1, p_.e_k2, p_.e_k3, p_.e_k4}};
    bcfg_.vertex = {{p_.v_k1, p_.v_k2, p_.v_k3, p_.v_k4}};
    bcfg_.yaw_k_rate = p_.y_k1;
    bcfg_.yaw_k_wheel = p_.y_k2;
    for (int k = 0; k < 3; ++k) bcfg_.kv[k] = p_.kv(k);
    bcfg_.u_limit = p_.u_limit;
    bcfg_.pos_limit = p_.p_limit;
    bcfg_.trim_tau_s = p_.trim_tau;
    bcfg_.trim_max_rad = p_.trim_max * cubli::kDegToRad;
    scfg_.arm_capture_rad = p_.arm_cap * cubli::kDegToRad;
    scfg_.arm_rate_rads = p_.arm_rate;
    scfg_.arm_hold_s = p_.arm_hold;
    scfg_.arm_timeout_s = p_.arm_tout;
    scfg_.tilt_limit_rad = p_.tilt_lim * cubli::kDegToRad;
    scfg_.rearm = p_.rearm > 0.5f;
}

void ControlLoop::abort_procedure() {
    if (procedure_ == Procedure::kAxes) axis_cal_.abort();
    if (procedure_ != Procedure::kNone && g_recording.busy.load() && !g_recording.ready.load()) {
        release_recording();
    }
    procedure_ = Procedure::kNone;
    drop_pending_ = false;
}

void ControlLoop::handle(const ControlCommand& c) {
    const int64_t now = esp_timer_get_time();
    const SysState st = sup_.state();
    switch (c.cmd) {
        case Cmd::kArm: req_ = SupRequest::kArm; break;
        case Cmd::kDisarm:
            req_ = SupRequest::kDisarm;
            abort_procedure();
            for (float& u : bench_u_) u = 0;
            break;
        case Cmd::kClearFault:
            req_ = SupRequest::kClearFault;
            if (st == SysState::kFault && !motors_->brake_released()) g_deadman_tripped.store(false);
            numeric_fault_ = false;
            overrun_leak_ = 0;
            break;
        case Cmd::kBenchOn: req_ = SupRequest::kBenchOn; bench_brake_engaged_ = true; break;
        case Cmd::kBenchOff:
            req_ = SupRequest::kBenchOff;
            for (float& u : bench_u_) u = 0;
            if (procedure_ == Procedure::kWireTest) procedure_ = Procedure::kNone;
            break;
        case Cmd::kBrake: bench_brake_engaged_ = c.i0 != 0; break;
        case Cmd::kMotor:
            if (st == SysState::kBench && c.i0 >= 0 && c.i0 < 3 && procedure_ == Procedure::kNone) {
                bench_u_[c.i0] = cubli::clampf(c.f0, -1.0f, 1.0f);
                bench_deadline_us_ = now + static_cast<int64_t>(kBenchHoldS * 1e6f);
                bench_timeout_posted_ = false;
            }
            break;
        case Cmd::kStop:
            for (float& u : bench_u_) u = 0;
            abort_procedure();
            if (sup_.armed() || st == SysState::kArming) req_ = SupRequest::kDisarm;
            break;
        case Cmd::kWireTest:
            if (st == SysState::kBench && procedure_ == Procedure::kNone && c.i0 >= 0 && c.i0 < 3) {
                procedure_ = Procedure::kWireTest;
                proc_motor_ = c.i0;
                proc_phase_ = 0;
                phase_start_us_ = proc_start_us_ = now;
                phase_count_[0] = enc_count_[c.i0];
            }
            break;
        case Cmd::kEncZero:
            for (int k = 0; k < 3; ++k) enc_zero_[k] = enc_count_[k];
            break;
        case Cmd::kCalibAxes:
            if ((st == SysState::kIdle || st == SysState::kReady) && procedure_ == Procedure::kNone) {
                procedure_ = Procedure::kAxes;
                proc_start_us_ = now;
                axis_cal_.start(cubli::AxisCalibrator::Config{});
            }
            break;
        case Cmd::kCalibPoint:
            if (!sup_.motors_allowed() && imu_ok_) {
                point_capture_ = true;
                point_start_us_ = now;
                point_sum_ = Vec3{};
                point_n_ = 0;
                point_rate_max_ = 0;
            } else {
                post(Ev::kPointRejected, imu_ok_ ? 1 : 0);
            }
            break;
        case Cmd::kCalibDeletePoint:
            if (!sup_.armed() && c.i0 >= 0 && c.i0 < cubli::kMaxBalancePoints) {
                cal_.points[c.i0] = cubli::BalancePoint{};
                publish_calib();
                post(Ev::kCalibChanged);
            }
            break;
        case Cmd::kCalibClear:
            if (!sup_.armed()) {
                for (auto& pt : cal_.points) pt = cubli::BalancePoint{};
                if (c.i0) cal_.axes_valid = 0;
                axes_ = cal_.motor_axes();
                publish_calib();
                post(Ev::kCalibChanged);
            }
            break;
        case Cmd::kSetCalib:
            if (!sup_.armed()) {
                cubli::CalibrationData nc;
                if (g_calib_in.read(nc)) {
                    cal_ = nc;
                    axes_ = cal_.motor_axes();
                    if (cal_.axes_valid && !axes_.valid) cal_.axes_valid = 0;
                    cubli::reclassify_points(cal_);
                    publish_calib();
                    post(Ev::kCalibChanged);
                }
            }
            break;
        case Cmd::kSysIdMotor:
            if ((st == SysState::kIdle || st == SysState::kReady) && procedure_ == Procedure::kNone &&
                c.i0 >= 0 && c.i0 < 3 && !g_recording.busy.load()) {
                g_recording.busy.store(true);
                g_recording.count = 0;
                g_recording.naxes = 1;
                g_recording.motor = c.i0;
                g_recording.procedure = Procedure::kSysIdMotor;
                procedure_ = Procedure::kSysIdMotor;
                proc_motor_ = c.i0;
                proc_start_us_ = now;
                rec_div_ = 0;
            }
            break;
        case Cmd::kAutotune:
            if (c.i0 == 0 && sup_.armed() && procedure_ == Procedure::kNone && !g_recording.busy.load()) {
                g_recording.busy.store(true);
                g_recording.count = 0;
                g_recording.naxes = sup_.state() == SysState::kArmedVertex ? 2 : 1;
                g_recording.motor = bal_.edge_motor();
                g_recording.point = sup_.active_point();
                g_recording.procedure = Procedure::kAutotune;
                procedure_ = Procedure::kAutotune;
                proc_start_us_ = now;
                rec_div_ = 0;
                rec_target_s_ = p_.at_time;
            } else if (c.i0 == 1 && st == SysState::kReady && procedure_ == Procedure::kNone) {
                drop_pending_ = true;
                req_ = SupRequest::kArm;
            }
            break;
        case Cmd::kAutotuneCancel: abort_procedure(); break;
        case Cmd::kLog:
            if (procedure_ == Procedure::kNone && !g_recording.busy.load()) {
                g_recording.busy.store(true);
                g_recording.count = 0;
                g_recording.naxes = sup_.state() == SysState::kArmedVertex ? 2 : 1;
                g_recording.motor = bal_.edge_motor();
                g_recording.point = sup_.active_point();
                g_recording.procedure = Procedure::kLog;
                procedure_ = Procedure::kLog;
                proc_start_us_ = now;
                rec_div_ = 0;
                rec_target_s_ = cubli::clampf(static_cast<float>(c.i0) * 1e-3f, 0.1f, 2.5f);
            }
            break;
        case Cmd::kImuSend: imu_->send_command(c.text); break;
        case Cmd::kImuRestart: imu_->restart_search(); break;
    }
}

void ControlLoop::finish_recording() {
    float kv = 0;
    for (int k = 0; k < 3; ++k) kv += p_.kv(k);
    g_recording.kv_mean = kv / 3.0f;
    g_recording.ready.store(true);
    post(Ev::kRecordDone, static_cast<int32_t>(g_recording.procedure), g_recording.count);
    procedure_ = Procedure::kNone;
}

void ControlLoop::record_balance(const cubli::BalanceOutput& out, const float u[3]) {
    if (++rec_div_ < 2) return;  // 500 Hz
    rec_div_ = 0;
    Recording& r = g_recording;
    if (r.count >= static_cast<int>(sizeof(r.samples) / sizeof(r.samples[0]))) return;
    cubli::SysIdSample& s = r.samples[r.count++];
    const int64_t now = esp_timer_get_time();
    s.dt = r.count > 1 ? static_cast<float>(now - last_rec_us_) * 1e-6f : 0.002f;
    last_rec_us_ = now;
    for (int j = 0; j < 2; ++j) {
        s.theta[j] = out.axis[j].theta;
        s.rate[j] = out.axis[j].rate;
    }
    if (bal_.mode() == BalanceMode::kEdge) {
        const int k = bal_.edge_motor();
        s.wheel_angle[0] = wheel_[k].angle(hw::kEncoderCountsPerRev);
        s.u[0] = u[k];
    } else {
        Vec3 ang, v;
        for (int k = 0; k < 3; ++k) {
            ang += axes_.axis[k] * wheel_[k].angle(hw::kEncoderCountsPerRev);
            v += axes_.axis[k] * u[k];
        }
        for (int j = 0; j < 2; ++j) {
            s.wheel_angle[j] = dot(bal_.basis(j), ang);
            s.u[j] = dot(bal_.basis(j), v);
        }
    }
}

void ControlLoop::enter_armed(SysState s) {
    const int idx = sup_.active_point();
    bool ok = idx >= 0 && axes_.valid;
    if (ok) {
        const cubli::BalancePoint& pt = cal_.points[idx];
        ok = (s == SysState::kArmedVertex) ? bal_.begin_vertex(pt.up, axes_) : bal_.begin_edge(pt.up, pt.motor, axes_);
    }
    if (!ok) req_ = SupRequest::kDisarm;
    if (ok && drop_pending_ && s == SysState::kArmedEdge && !g_recording.busy.load()) {
        drop_pending_ = false;
        g_recording.busy.store(true);
        g_recording.count = 0;
        g_recording.naxes = 1;
        g_recording.motor = bal_.edge_motor();
        g_recording.point = idx;
        g_recording.procedure = Procedure::kAutotuneDrop;
        procedure_ = Procedure::kAutotuneDrop;
        proc_start_us_ = esp_timer_get_time();
        rec_div_ = 0;
        rec_target_s_ = 0.8f;
    }
}

void ControlLoop::compute_outputs(int64_t now_us, float dt, float u[3], bool& want_release) {
    for (int k = 0; k < 3; ++k) u[k] = 0.0f;
    want_release = false;
    brake_test_ = false;
    const SysState st = sup_.state();
    const float t_proc = static_cast<float>(now_us - proc_start_us_) * 1e-6f;

    // PRBS excitation, 50 Hz chip rate.
    if (++prbs_div_ >= 20) {
        prbs_div_ = 0;
        prbs_v_[0] = prbs0_.next(p_.at_amp);
        prbs_v_[1] = prbs1_.next(p_.at_amp);
    }

    if (sup_.armed()) {
        want_release = true;
        cubli::BalanceInput in;
        in.up = up_;
        in.gyro = gyro_;
        in.dt = dt;
        for (int k = 0; k < 3; ++k) in.wheel[k] = wheel_speed_[k];
        last_out_ = bal_.step(in, bcfg_);

        if (procedure_ == Procedure::kAutotuneDrop) {
            // Open loop: excitation only, until the cube has fallen 8 deg.
            const int k = bal_.edge_motor();
            if (k >= 0) u[k] = prbs_v_[0] * 2.0f;
            record_balance(last_out_, u);
            if (last_out_.tilt_rad > 8.0f * cubli::kDegToRad || t_proc > rec_target_s_) {
                finish_recording();
                req_ = SupRequest::kDisarm;
            }
            return;
        }
        for (int k = 0; k < 3; ++k) u[k] = last_out_.u[k];
        if (procedure_ == Procedure::kAutotune) {
            if (bal_.mode() == BalanceMode::kEdge) {
                u[bal_.edge_motor()] += prbs_v_[0];
            } else {
                const Vec3 v = bal_.basis(0) * prbs_v_[0] + bal_.basis(1) * prbs_v_[1];
                const Vec3 t = axes_.inv * v;
                for (int k = 0; k < 3; ++k) u[k] += t[k];
            }
        }
        for (int k = 0; k < 3; ++k) {
            if (!std::isfinite(u[k])) numeric_fault_ = true;
            u[k] = cubli::clampf(std::isfinite(u[k]) ? u[k] : 0.0f, -p_.u_limit, p_.u_limit);
        }
        if (procedure_ == Procedure::kAutotune || procedure_ == Procedure::kLog) {
            record_balance(last_out_, u);
            if (t_proc >= rec_target_s_) finish_recording();
        }
        return;
    }

    if (procedure_ == Procedure::kLog) {  // logging while not armed: attitude + wheels only
        cubli::BalanceOutput o;
        o.axis[0].theta = up_.x;
        o.axis[1].theta = up_.y;
        o.axis[0].rate = gyro_.x;
        o.axis[1].rate = gyro_.y;
        record_balance(o, u);
        if (t_proc >= rec_target_s_) finish_recording();
    }

    if (st == SysState::kBench) {
        want_release = !bench_brake_engaged_;
        if (procedure_ == Procedure::kWireTest) {
            const int k = proc_motor_;
            const float tp = static_cast<float>(now_us - phase_start_us_) * 1e-6f;
            static constexpr float kDur[5] = {1.0f, 1.0f, 1.0f, 1.0f, 0.6f};
            static constexpr float kCmd[5] = {0.15f, 0.0f, -0.15f, 0.0f, 0.15f};
            u[k] = kCmd[proc_phase_];
            want_release = proc_phase_ < 4;  // last phase: braked
            brake_test_ = proc_phase_ == 4;
            if (proc_phase_ == 0) wiretest_speed_ = wheel_speed_[k];
            if (tp >= kDur[proc_phase_]) {
                phase_count_[proc_phase_ + 1] = enc_count_[k];
                phase_start_us_ = now_us;
                if (++proc_phase_ >= 5) {
                    procedure_ = Procedure::kNone;
                    post(Ev::kWireTestDone, k, 0, static_cast<float>(phase_count_[1] - phase_count_[0]),
                         static_cast<float>(phase_count_[3] - phase_count_[2]),
                         static_cast<float>(phase_count_[5] - phase_count_[4]), wiretest_speed_);
                    bench_brake_engaged_ = true;
                    u[k] = 0;
                }
            }
            return;
        }
        if (now_us < bench_deadline_us_) {
            for (int k = 0; k < 3; ++k) u[k] = bench_u_[k];
        } else {
            for (float& b : bench_u_) b = 0;
            if (!bench_timeout_posted_) {
                bench_timeout_posted_ = true;
                post(Ev::kBenchTimeout);
            }
        }
        return;
    }

    if (st == SysState::kCalibrate) {
        want_release = true;
        if (procedure_ == Procedure::kAxes) {
            axis_cal_.step(dt, gyro_, u);
            if (axis_cal_.status() == cubli::AxisCalibrator::Status::kDone) {
                const cubli::MotorAxes& r = axis_cal_.result();
                for (int k = 0; k < 3; ++k) cal_.axis[k] = r.axis[k];
                cal_.axes_valid = 1;
                axes_ = cal_.motor_axes();
                cubli::reclassify_points(cal_);
                publish_calib();
                post(Ev::kAxesDone, 1, 0, axis_cal_.delta_magnitude(0), axis_cal_.delta_magnitude(1),
                     axis_cal_.delta_magnitude(2));
                post(Ev::kCalibChanged);
                procedure_ = Procedure::kNone;
            } else if (axis_cal_.status() == cubli::AxisCalibrator::Status::kFailed) {
                post(Ev::kAxesFailed, axis_cal_.current_motor());
                procedure_ = Procedure::kNone;
            }
        } else if (procedure_ == Procedure::kSysIdMotor) {
            // Staircase on one motor, cube resting on a face (body fixed).
            static constexpr float kT[4] = {0.6f, 1.2f, 1.8f, 2.5f};
            static constexpr float kU[4] = {0.15f, 0.45f, 0.15f, 0.30f};
            int i = 0;
            while (i < 3 && t_proc > kT[i]) ++i;
            u[proc_motor_] = kU[i];
            if (++rec_div_ >= 2) {
                rec_div_ = 0;
                Recording& r = g_recording;
                if (r.count < static_cast<int>(sizeof(r.samples) / sizeof(r.samples[0]))) {
                    cubli::SysIdSample& s = r.samples[r.count++];
                    s.dt = 0.002f;
                    s.wheel_angle[0] = wheel_[proc_motor_].angle(hw::kEncoderCountsPerRev);
                    s.u[0] = motors_->command(proc_motor_);
                }
            }
            if (t_proc >= kT[3]) {
                u[proc_motor_] = 0.0f;
                finish_recording();
            }
        }
        return;
    }
}

void ControlLoop::apply_outputs(const float u_in[3], bool want_release, float dt) {
    (void)dt;
    const int64_t now = esp_timer_get_time();
    const bool active = want_release;
    if (was_active_ && !active) inactive_since_us_ = now;
    was_active_ = active;

    bool release = want_release;
    if (!active) {
        // Stop strategy (context.md §5.3 + owner decision D): PWM at stop at once;
        // keep BRAKE released (coast) while a wheel is still fast, at most coast_s.
        const float since = static_cast<float>(now - inactive_since_us_) * 1e-6f;
        release = motors_->brake_released() && max_wheel_speed() > p_.brake_rads && since < p_.coast_s;
    }
    if (g_deadman_tripped.load()) release = false;  // the timer already braked: stay braked

    float u[3];
    for (int k = 0; k < 3; ++k) {
        u[k] = active ? u_in[k] : 0.0f;
        if (std::fabs(u[k]) < p_.u_deadband) u[k] = 0.0f;
        // Direction reversal only below dir_sw_rads (vendor: stop before reversing).
        const bool inv = p_.dir_inv(k) > 0.5f;
        const int want_level = (u[k] > 0.0f ? 1 : 0) ^ (inv ? 1 : 0);
        if (u[k] != 0.0f && want_level != motors_->dir_level(k) && std::fabs(wheel_speed_[k]) > p_.dir_sw_rads)
            u[k] = 0.0f;
    }

    if (brake_test_ && !g_deadman_tripped.load() && sup_.state() == SysState::kBench) {
        // Brake engaged first, then PWM: the wheel must not turn (checks the yellow wire).
        motors_->set_brake_released(false);
        for (int k = 0; k < 3; ++k) motors_->set(k, u_in[k], p_.dir_inv(k) > 0.5f);
    } else if (!release) {
        motors_->stop_all();
        motors_->set_brake_released(false);
    } else if (!motors_->brake_released()) {
        // Release only from the stop level; commands start next cycle.
        motors_->stop_all();
        motors_->set_brake_released(true);
    } else {
        for (int k = 0; k < 3; ++k) motors_->set(k, u[k], p_.dir_inv(k) > 0.5f);
    }
    g_motors_live.store(motors_->brake_released());
}

void ControlLoop::step(int64_t now_us, bool new_frame) {
    float dt = last_us_ ? static_cast<float>(now_us - last_us_) * 1e-6f : 0.001f;
    const uint32_t period = static_cast<uint32_t>(now_us - last_us_);
    last_us_ = now_us;
    dt = cubli::clampf(dt, 1e-4f, 0.05f);

    refresh_params();
    ControlCommand c;
    while (xQueueReceive(g_cmd_queue, &c, 0) == pdTRUE) handle(c);

    // ---- IMU ----
    if (new_frame) {
        const cubli::ImuFrameV2& f = imu_->latest();
        imu_status_ = f.status;
        up_ = cubli::up_in_body(f.q);
        gyro_ = f.gyro;
        accel_ = f.accel;
    }
    imu_age_us_ = imu_->last_v2_us() ? now_us - imu_->last_v2_us() : 1000000000;
    imu_ok_ = imu_->state() == hal::ImuLink::State::kV2 && (imu_status_ & cubli::imu_status::kEkfReady) &&
              imu_age_us_ < kImuReadyMaxAgeUs;
    if (imu_->state() != last_link_state_) {
        last_link_state_ = imu_->state();
        post(Ev::kImuLinkChanged, static_cast<int32_t>(last_link_state_));
    }

    // ---- encoders ----
    for (int k = 0; k < 3; ++k) {
        const int32_t raw = enc_->count(k);
        enc_count_[k] = p_.enc_inv(k) > 0.5f ? -raw : raw;
        wheel_speed_[k] = wheel_[k].update(enc_count_[k], static_cast<uint32_t>(now_us), static_cast<int>(p_.wheel_win),
                                           p_.wheel_fc, hw::kEncoderCountsPerRev);
    }

    g_power.read(power_);

    // ---- balance point capture (averaged, cube must be still) ----
    if (point_capture_) {
        point_sum_ += up_;
        ++point_n_;
        point_rate_max_ = std::fmax(point_rate_max_, norm(gyro_));
        if (now_us - point_start_us_ > 500000) {
            point_capture_ = false;
            if (point_rate_max_ > 0.1f || !imu_ok_) {
                post(Ev::kPointRejected, 2);
            } else {
                const Vec3 up = normalized(point_sum_);
                int motor = -1;
                const PointKind kind = cubli::classify_point(axes_, up, &motor);
                const int slot = cubli::store_point(cal_, up, kind, motor, 10.0f * cubli::kDegToRad);
                if (slot < 0) {
                    post(Ev::kPointRejected, 3);
                } else {
                    publish_calib();
                    post(Ev::kPointStored, slot, static_cast<int32_t>(kind), static_cast<float>(motor));
                    post(Ev::kCalibChanged);
                }
            }
        }
    }

    // ---- supervisor ----
    sin_.dt = dt;
    sin_.imu_ok = imu_ok_;
    sin_.battery_present = power_.sensor_ok && power_.present;
    sin_.battery_ok = power_.above_cutoff;
    uint32_t cond = 0;
    if (power_.present && !power_.above_cutoff) cond |= cubli::fault::kBatteryLow;
    if (power_.overcurrent) cond |= cubli::fault::kOverCurrent;
    if ((sup_.armed() || procedure_ == Procedure::kAxes) && imu_age_us_ > static_cast<int64_t>(p_.imu_tout) * 1000)
        cond |= cubli::fault::kImuLost;
    if (max_wheel_speed() > p_.wheel_max) cond |= cubli::fault::kWheelOverspeed;
    if (g_deadman_tripped.load()) cond |= cubli::fault::kDeadman;
    if (overrun_leak_ > 10.0f) cond |= cubli::fault::kOverrun;
    if (numeric_fault_) cond |= cubli::fault::kNumeric;
    sin_.fault_conditions = cond;
    sin_.axes_valid = axes_.valid;
    sin_.point_count = cal_.point_count();
    sin_.calibrating = calibrating();
    float ang = 1e9f;
    const int near = imu_ok_ ? cubli::find_nearest_point(cal_, up_, static_cast<int>(p_.mode), &ang) : -1;
    sin_.nearest_point = near;
    sin_.nearest_angle = ang;
    sin_.nearest_is_edge = near >= 0 && cal_.points[near].kind == PointKind::kEdge;
    sin_.nearest_is_vertex = near >= 0 && cal_.points[near].kind == PointKind::kVertex;
    sin_.body_rate = norm(gyro_);
    sin_.tilt_rad = sup_.armed() ? last_out_.tilt_rad : 0.0f;

    const SysState before = sup_.state();
    const SupEvent ev = sup_.update(sin_, req_, scfg_);
    req_ = SupRequest::kNone;
    const SysState after = sup_.state();
    if (ev != SupEvent::kNone) {
        ControlEvent e;
        e.ev = Ev::kSupervisor;
        e.i0 = static_cast<int32_t>(ev);
        e.i1 = static_cast<int32_t>(after);
        g_events.push(e);
        g_ui_events.push(e);
    }
    const bool was_armed = before == SysState::kArmedEdge || before == SysState::kArmedVertex;
    if (after != before) {
        if (after == SysState::kArmedEdge || after == SysState::kArmedVertex) enter_armed(after);
        if (was_armed && !sup_.armed()) {
            bal_.stop();
            if (procedure_ == Procedure::kAutotune || procedure_ == Procedure::kLog) abort_procedure();
            if (procedure_ == Procedure::kAutotuneDrop) finish_recording();
        }
        if (after == SysState::kFault) {
            if (procedure_ == Procedure::kAutotuneDrop && g_recording.count > 50) finish_recording();
            abort_procedure();
            for (float& u : bench_u_) u = 0;
        }
        if (before == SysState::kBench) {
            for (float& u : bench_u_) u = 0;
            if (procedure_ == Procedure::kWireTest) procedure_ = Procedure::kNone;
        }
        if (before == SysState::kCalibrate && calibrating()) abort_procedure();
        if (after == SysState::kReady || after == SysState::kIdle) drop_pending_ = false;
    }

    // ---- outputs ----
    float u[3];
    bool want_release = false;
    compute_outputs(now_us, dt, u, want_release);
    apply_outputs(u, want_release, dt);

    // ---- telemetry ----
    const int tlm_hz = static_cast<int>(std::fmax(p_.tlm_hz, p_.ble_tlm_hz));
    if (tlm_hz > 0 && now_us - last_tlm_us_ >= 1000000 / tlm_hz) {
        last_tlm_us_ = now_us;
        TelemetrySample t;
        t.t_ms = static_cast<uint32_t>(now_us / 1000);
        t.state = static_cast<uint8_t>(after);
        t.brake = motors_->brake_released() ? 0 : 1;
        t.exec_us = static_cast<uint16_t>(last_exec_us_ > 65535 ? 65535 : last_exec_us_);
        for (int j = 0; j < 2; ++j) {
            t.theta[j] = sup_.armed() ? last_out_.axis[j].theta : 0.0f;
            t.rate[j] = sup_.armed() ? last_out_.axis[j].rate : 0.0f;
        }
        for (int k = 0; k < 3; ++k) {
            t.wheel[k] = wheel_speed_[k];
            t.u[k] = motors_->command(k);
        }
        t.volts = power_.v_inst;
        t.amps = power_.i_inst;
        g_telemetry.push(t);
    }

    // ---- loop statistics ----
    if (period > 0 && period < 1000000) {
        if (period > period_max_) period_max_ = period;
        if (period < period_min_) period_min_ = period;
    }
    ++steps_in_window_;
    if (now_us - stats_window_us_ >= 1000000) {
        loop_hz_ = static_cast<float>(steps_in_window_) * 1e6f / static_cast<float>(now_us - stats_window_us_);
        pub_exec_max_ = exec_max_;
        pub_period_max_ = period_max_;
        pub_period_min_ = period_min_;
        exec_max_ = period_max_ = 0;
        period_min_ = 0xFFFFFFFF;
        steps_in_window_ = 0;
        stats_window_us_ = now_us;
    }
    if (now_us - last_status_us_ >= 10000) {
        last_status_us_ = now_us;
        publish_status(now_us);
    }
}

void ControlLoop::publish_status(int64_t now_us) {
    ControlStatus s;
    s.t_ms = static_cast<uint32_t>(now_us / 1000);
    s.state = sup_.state();
    s.faults = sup_.faults();
    s.fault_conditions = sin_.fault_conditions;
    s.active_point = static_cast<int8_t>(sup_.active_point());
    s.nearest_point = static_cast<int8_t>(sin_.nearest_point);
    s.nearest_deg = sin_.nearest_point >= 0 ? sin_.nearest_angle * cubli::kRadToDeg : 0.0f;
    std::strncpy(s.not_ready, sup_.not_ready_reason(sin_), sizeof(s.not_ready) - 1);
    s.imu_state = static_cast<uint8_t>(imu_->state());
    s.imu_baud = imu_->baud();
    s.imu_status = imu_status_;
    s.imu_ok = imu_ok_;
    s.imu_frames_v1 = imu_->parser().frames_v1();
    s.imu_frames_v2 = imu_->parser().frames_v2();
    s.imu_crc_errors = imu_->parser().crc_errors();
    s.imu_gaps = imu_->parser().seq_gaps();
    s.imu_overflows = imu_->uart_overflows();
    s.imu_age_ms = static_cast<float>(imu_age_us_) * 1e-3f;
    s.q = imu_->latest().q;
    s.up = up_;
    s.gyro = gyro_;
    s.accel = accel_;
    s.v1_roll = imu_->latest_v1().roll_deg;
    s.v1_pitch = imu_->latest_v1().pitch_deg;
    s.v1_yaw = imu_->latest_v1().yaw_deg;
    s.imu_supply_v = imu_->latest_v1().supply_v;
    for (int k = 0; k < 3; ++k) {
        s.enc_raw[k] = enc_->count(k);
        s.enc_rel[k] = enc_count_[k] - enc_zero_[k];
        s.wheel[k] = wheel_speed_[k];
        s.u[k] = motors_->command(k);
    }
    s.brake_released = motors_->brake_released();
    s.tilt_deg = sup_.armed() ? last_out_.tilt_rad * cubli::kRadToDeg : 0.0f;
    for (int j = 0; j < 2; ++j) s.theta_deg[j] = sup_.armed() ? last_out_.axis[j].theta * cubli::kRadToDeg : 0.0f;
    s.loop_hz = loop_hz_;
    s.exec_us_max = pub_exec_max_;
    s.period_us_max = pub_period_max_;
    s.period_us_min = pub_period_min_;
    s.overruns = overruns_;
    s.deadman_tripped = g_deadman_tripped.load();
    s.procedure = static_cast<uint8_t>(procedure_);
    s.procedure_progress = procedure_ != Procedure::kNone
                               ? static_cast<float>(now_us - proc_start_us_) * 1e-6f
                               : 0.0f;
    g_status.write(s);
}

void ControlLoop::run() {
    esp_task_wdt_add(nullptr);
    axes_ = cal_.motor_axes();
    publish_calib();
    const int64_t t0 = esp_timer_get_time();
    for (int k = 0; k < 3; ++k) wheel_[k].reset(enc_->count(k), static_cast<uint32_t>(t0));
    stats_window_us_ = t0;

    esp_timer_handle_t deadman = nullptr;
    esp_timer_create_args_t ta = {};
    ta.callback = deadman_cb;
    ta.name = "deadman";
    esp_timer_create(&ta, &deadman);
    esp_timer_start_periodic(deadman, 2000);

    for (;;) {
        const bool frame = imu_->poll(kIdlePollUs, esp_timer_get_time());
        const int64_t start = esp_timer_get_time();
        step(start, frame);
        const int64_t end = esp_timer_get_time();
        last_exec_us_ = static_cast<uint32_t>(end - start);
        if (last_exec_us_ > exec_max_) exec_max_ = last_exec_us_;
        if (static_cast<float>(last_exec_us_) > kOverrunExecUs) {
            ++overruns_;
            overrun_leak_ += 1.0f;
        } else if (overrun_leak_ > 0.0f) {
            overrun_leak_ -= 0.01f;
        }
        g_control_heartbeat_us.store(end);
        esp_task_wdt_reset();
    }
}

}  // namespace

void start_control_task(hal::MotorOutput* motors, hal::Encoders* enc, hal::ImuLink* imu,
                        const cubli::CalibrationData& calib) {
    static ControlLoop* loop = new ControlLoop(motors, enc, imu, calib);
    g_control_heartbeat_us.store(esp_timer_get_time());
    xTaskCreatePinnedToCore([](void* arg) { static_cast<ControlLoop*>(arg)->run(); }, "control", 8192, loop,
                            configMAX_PRIORITIES - 2, nullptr, 1);
}

}  // namespace app
