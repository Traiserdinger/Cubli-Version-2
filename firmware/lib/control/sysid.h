// Least-squares identification of the per-axis model used by the LQR design
// (see lqr.h) from a recorded experiment.
//
//   theta_ddot = a*theta + b*(u - kv*s)
//   s_dot      = a*theta + (b + c)*(u - kv*s)
//
// u is the motor command actually applied along the axis (after saturation,
// including back-EMF compensation). The wheel speed s is derived here from the
// raw wheel angle (not from the controller's filtered estimate, whose lag would
// bias the fit). All signals are filtered forward-backward (zero phase) and
// differentiated with central differences.
#pragma once

#include <cstdint>

namespace cubli {

inline constexpr int kSysIdMaxSamples = 1500;

struct SysIdSample {
    float dt = 0;
    float theta[2] = {0, 0};
    float rate[2] = {0, 0};
    float wheel_angle[2] = {0, 0};  // raw encoder angle along the axis (rad)
    float u[2] = {0, 0};
};

struct SysIdResult {
    bool ok = false;
    float a = 0, b = 0, c = 0, kv = 0;
    float r2_body = 0, r2_wheel = 0;
    int samples = 0;
    const char* error = "";
};

/// naxes = 1 (edge) or 2 (vertex: both horizontal axes are stacked into one fit).
/// kv_known < 0: kv is estimated as well.
SysIdResult identify_axis(const SysIdSample* data, int n, int naxes, float filter_hz, float kv_known);

struct MotorIdResult {
    bool ok = false;
    float c = 0, kv = 0, r2 = 0;
    const char* error = "";
};

/// Body held fixed (cube resting on a face): s_dot = c*(u - kv*s).
MotorIdResult identify_motor(const float* s, const float* u, int n, float dt, float filter_hz);

}  // namespace cubli
