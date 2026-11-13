// Balance controller: edge (one wheel) and vertex (three wheels).
//
// Frames and signs (all vectors in the IMU PCB body frame):
//   up        world +Z in body coordinates (from the IMU quaternion)
//   up_eq     calibrated "up" at the balance point
//   tilt      = up x up_eq   = small body rotation away from equilibrium (rad)
//   axis[k]   unit vector: direction of body angular acceleration produced by a
//             POSITIVE command on motor k (found by `calib axes`)
//   wheel[k]  wheel k speed relative to the body, positive for positive command
//             (guaranteed by enc_invert, set with `wiretest`)
//
// Each balancing axis j (edge: the motor axis; vertex: two horizontal axes e1,
// e2) uses the state x_j = [theta_j, rate_j, s_j, p_j] and the law
// tau_j = -K x_j (K from LQR, see lqr.h). The yaw axis at the vertex only damps
// body spin and wheel momentum. Per-motor output u_k = tau_k + kv_k * wheel_k
// cancels back-EMF, then saturates.
#pragma once

#include <cstdint>

#include "math3.h"

namespace cubli {

inline constexpr int kMotors = 3;

struct MotorAxes {
    Vec3 axis[kMotors] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    Mat3 inv;  // inverse of [axis0 axis1 axis2]
    bool valid = false;

    /// Normalises the axes and computes the mixing inverse. Requires the axes to be
    /// close to orthogonal (|cos| < max_cos between any pair).
    bool finalize(float max_cos = 0.3f);
};

struct AxisGains {
    float k[4] = {0, 0, 0, 0};  // theta, rate, wheel speed, wheel angle
};

struct BalanceConfig {
    AxisGains edge;
    AxisGains vertex;
    float yaw_k_rate = 0.0f;
    float yaw_k_wheel = 0.0f;
    float kv[kMotors] = {0, 0, 0};  // back-EMF compensation, command per rad/s
    float u_limit = 1.0f;           // |u_k| <= u_limit <= 1
    float pos_limit = 100.0f;       // rad, clamp of the wheel-angle state (anti-windup)
    float trim_tau_s = 0.0f;        // equilibrium self-trim time constant, 0 = off
    float trim_max_rad = 0.05f;     // max trim away from the calibrated point
};

enum class BalanceMode : uint8_t { kOff = 0, kEdge = 1, kVertex = 2 };

struct BalanceInput {
    Vec3 up;
    Vec3 gyro;  // rad/s, body frame, bias-corrected
    float wheel[kMotors] = {0, 0, 0};
    float dt = 0.001f;
};

struct AxisState {
    float theta = 0, rate = 0, s = 0, p = 0, tau = 0, u_axis = 0;
};

struct BalanceOutput {
    float u[kMotors] = {0, 0, 0};
    AxisState axis[2];  // edge uses axis[0]
    float yaw_rate = 0, yaw_wheel = 0;
    float tilt_rad = 0;  // angle between up and the (trimmed) equilibrium
    bool saturated = false;
};

class BalanceController {
public:
    bool begin_edge(Vec3 up_eq, int motor, const MotorAxes& axes);
    bool begin_vertex(Vec3 up_eq, const MotorAxes& axes);
    void stop() { mode_ = BalanceMode::kOff; }

    BalanceOutput step(const BalanceInput& in, const BalanceConfig& cfg);

    BalanceMode mode() const { return mode_; }
    int edge_motor() const { return motor_; }
    Vec3 equilibrium() const { return up_eq_; }
    Vec3 trimmed_equilibrium() const { return up_trim_; }
    /// Horizontal basis used at the vertex (for logging / system identification).
    Vec3 basis(int j) const { return e_[j]; }

private:
    float axis_law(AxisState& st, const AxisGains& g) const;

    BalanceMode mode_ = BalanceMode::kOff;
    MotorAxes axes_;
    int motor_ = -1;
    Vec3 up_eq_, up_trim_;
    Vec3 e_[2];
    float p_[2] = {0, 0};
};

}  // namespace cubli
