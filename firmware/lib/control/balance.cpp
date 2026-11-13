#include "balance.h"

#include <cmath>

namespace cubli {

bool MotorAxes::finalize(float max_cos) {
    valid = false;
    for (auto& a : axis) {
        if (!is_finite(a) || norm(a) < 1e-3f) return false;
        a = normalized(a);
    }
    for (int i = 0; i < kMotors; ++i)
        for (int j = i + 1; j < kMotors; ++j)
            if (std::fabs(dot(axis[i], axis[j])) > max_cos) return false;
    if (!Mat3::from_columns(axis[0], axis[1], axis[2]).inverse(inv)) return false;
    valid = true;
    return true;
}

bool BalanceController::begin_edge(Vec3 up_eq, int motor, const MotorAxes& axes) {
    if (!axes.valid || motor < 0 || motor >= kMotors || norm(up_eq) < 0.5f) return false;
    axes_ = axes;
    motor_ = motor;
    up_eq_ = up_trim_ = normalized(up_eq);
    p_[0] = p_[1] = 0.0f;
    mode_ = BalanceMode::kEdge;
    return true;
}

bool BalanceController::begin_vertex(Vec3 up_eq, const MotorAxes& axes) {
    if (!axes.valid || norm(up_eq) < 0.5f) return false;
    axes_ = axes;
    motor_ = -1;
    up_eq_ = up_trim_ = normalized(up_eq);
    // Horizontal basis: e0 along the motor-0 axis projected on the horizontal
    // plane, e1 completes a right-handed set with up_eq.
    Vec3 h = axes.axis[0] - up_eq_ * dot(axes.axis[0], up_eq_);
    if (norm(h) < 0.1f) h = axes.axis[1] - up_eq_ * dot(axes.axis[1], up_eq_);
    e_[0] = normalized(h);
    e_[1] = cross(up_eq_, e_[0]);
    p_[0] = p_[1] = 0.0f;
    mode_ = BalanceMode::kVertex;
    return true;
}

float BalanceController::axis_law(AxisState& st, const AxisGains& g) const {
    st.tau = -(g.k[0] * st.theta + g.k[1] * st.rate + g.k[2] * st.s + g.k[3] * st.p);
    return st.tau;
}

BalanceOutput BalanceController::step(const BalanceInput& in, const BalanceConfig& cfg) {
    BalanceOutput out;
    if (mode_ == BalanceMode::kOff) return out;
    const float dt = in.dt > 0.0f ? in.dt : 0.001f;
    const Vec3 up = normalized(in.up);

    // Equilibrium self-trim: the long-run mean of "up" while balancing is the true
    // balance point (gravity torque averages to zero). Limited around up_eq_.
    if (cfg.trim_tau_s > 0.0f) {
        Vec3 t = normalized(up_trim_ + (up - up_trim_) * (dt / (cfg.trim_tau_s + dt)));
        const float off = angle_between(t, up_eq_);
        if (off > cfg.trim_max_rad && off > 1e-6f) {
            // Pull back onto the cone of half-angle trim_max around up_eq_.
            const Vec3 perp = normalized(t - up_eq_ * dot(t, up_eq_));
            t = up_eq_ * std::cos(cfg.trim_max_rad) + perp * std::sin(cfg.trim_max_rad);
        }
        up_trim_ = t;
    } else {
        up_trim_ = up_eq_;
    }

    const Vec3 tilt = cross(up, up_trim_);
    out.tilt_rad = angle_between(up, up_trim_);
    const float lim = cfg.u_limit > 0.0f && cfg.u_limit <= 1.0f ? cfg.u_limit : 1.0f;
    float tau[kMotors] = {0, 0, 0};

    if (mode_ == BalanceMode::kEdge) {
        const Vec3 n = axes_.axis[motor_];
        AxisState& st = out.axis[0];
        st.theta = dot(n, tilt);
        st.rate = dot(n, in.gyro);
        st.s = in.wheel[motor_];
        p_[0] = clampf(p_[0] + st.s * dt, -cfg.pos_limit, cfg.pos_limit);
        st.p = p_[0];
        tau[motor_] = axis_law(st, cfg.edge);
        st.u_axis = tau[motor_];
    } else {
        // Wheel-speed vector expressed along the body-acceleration directions.
        Vec3 S;
        for (int k = 0; k < kMotors; ++k) S += axes_.axis[k] * in.wheel[k];
        Vec3 v;
        for (int j = 0; j < 2; ++j) {
            AxisState& st = out.axis[j];
            st.theta = dot(e_[j], tilt);
            st.rate = dot(e_[j], in.gyro);
            st.s = dot(e_[j], S);
            p_[j] = clampf(p_[j] + st.s * dt, -cfg.pos_limit, cfg.pos_limit);
            st.p = p_[j];
            v += e_[j] * axis_law(st, cfg.vertex);
        }
        out.yaw_rate = dot(up_trim_, in.gyro);
        out.yaw_wheel = dot(up_trim_, S);
        v += up_trim_ * -(cfg.yaw_k_rate * out.yaw_rate + cfg.yaw_k_wheel * out.yaw_wheel);
        const Vec3 t = axes_.inv * v;  // solve sum_k tau_k axis_k = v
        for (int k = 0; k < kMotors; ++k) tau[k] = t[k];
        for (int j = 0; j < 2; ++j) out.axis[j].u_axis = dot(e_[j], v);
    }

    for (int k = 0; k < kMotors; ++k) {
        const bool active = (mode_ == BalanceMode::kVertex) || (k == motor_);
        if (!active) continue;
        float u = tau[k] + cfg.kv[k] * in.wheel[k];
        if (u > lim) { u = lim; out.saturated = true; }
        if (u < -lim) { u = -lim; out.saturated = true; }
        out.u[k] = u;
    }
    return out;
}

}  // namespace cubli
