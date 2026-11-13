// LQR design for one balancing axis of the cube (edge, or one horizontal axis
// at the vertex).
//
// Per-axis linear model (derived in firmware/README.md "Control design"), with
// the measured back-EMF compensated in the output stage (u = tau + kv*s):
//
//   x = [theta, theta_dot, s, p]     theta: tilt from equilibrium (rad)
//                                    s:     wheel speed relative to body (rad/s)
//                                    p:     wheel angle since arming (rad)
//   theta_ddot = a*theta + b*tau
//   s_dot      = a*theta + (b + c)*tau
//   p_dot      = s
//
//   a = M g l / J   (gravity),  b = Km / J  (motor on body),  c = Km / Iw (motor on wheel)
//
// tau is the normalised motor command after back-EMF compensation. The control
// law is tau = -K x. The infinite-horizon discrete LQR is solved with the
// structure-preserving doubling algorithm (quadratic convergence, ~20 steps).
#pragma once

#include "linalg.h"

namespace cubli {

struct AxisModel {
    float a = 0.0f;  // 1/s^2
    float b = 0.0f;  // rad/s^2 per unit command
    float c = 0.0f;  // rad/s^2 per unit command
};

/// Bryson-rule weights: Q = diag(1/x_max^2), R = 1/u_max^2.
struct LqrWeights {
    float theta_max = 0.05f;   // rad
    float rate_max = 1.0f;     // rad/s
    float wheel_max = 300.0f;  // rad/s
    float pos_max = 300.0f;    // rad
    float u_max = 0.5f;
};

struct LqrResult {
    bool ok = false;
    float k[4] = {0, 0, 0, 0};
    double closed_loop_norm = 0.0;  // ||(A-BK)^(2^20)||, ~0 when stable
    int iterations = 0;
};

void axis_model_matrices(const AxisModel& m, Mat<4, 4>& A, Mat<4, 1>& B);
void discretize(const Mat<4, 4>& A, const Mat<4, 1>& B, double dt, Mat<4, 4>& Ad, Mat<4, 1>& Bd);

/// Solves the DARE and returns the optimal gain. ok == false if the model is not
/// physically plausible (a, b, c must be > 0) or the solution is not stabilising.
LqrResult design_axis_lqr(const AxisModel& model, const LqrWeights& w, float dt);

/// Closed-loop check of an arbitrary gain on a model (used for manual gains).
bool axis_gain_stabilises(const AxisModel& model, const float k[4], float dt, double* norm_out = nullptr);

}  // namespace cubli
