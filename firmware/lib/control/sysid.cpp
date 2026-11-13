#include "sysid.h"

#include <cmath>

#include "linalg.h"

namespace cubli {
namespace {

// Scratch storage (identification runs in one task, never concurrently). Heap
// allocated on first use so it does not occupy static DRAM on the ESP32;
// identification never runs inside the control loop.
struct Scratch {
    float t[kSysIdMaxSamples];
    float x[4][kSysIdMaxSamples];  // theta, rate, s, u (filtered)
};
Scratch& scratch() {
    static Scratch* s = new Scratch();
    return *s;
}
#define g_t (scratch().t)
#define g_x (scratch().x)

void filtfilt(float* x, int n, const float* t, float fc) {
    if (fc <= 0.0f || n < 3) return;
    const float rc = 1.0f / (2.0f * 3.14159265f * fc);
    float y = x[0];
    for (int i = 1; i < n; ++i) {
        const float dt = t[i] - t[i - 1];
        y += (dt / (rc + dt)) * (x[i] - y);
        x[i] = y;
    }
    y = x[n - 1];
    for (int i = n - 2; i >= 0; --i) {
        const float dt = t[i + 1] - t[i];
        y += (dt / (rc + dt)) * (x[i] - y);
        x[i] = y;
    }
}

/// Normal-equation accumulator for y = sum_j beta_j * phi_j (up to 3 regressors).
struct LeastSquares {
    int m = 0;
    Mat<3, 3> ata;
    Mat<3, 1> aty;
    double yy = 0, ysum = 0;
    int n = 0;

    void add(const double* phi, double y) {
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < m; ++j) ata(i, j) += phi[i] * phi[j];
            aty(i, 0) += phi[i] * y;
        }
        yy += y * y;
        ysum += y;
        ++n;
    }
    bool solve(double* beta, double* r2) const {
        Mat<3, 3> A = ata;
        for (int i = m; i < 3; ++i) A(i, i) = 1.0;  // unused regressors
        Mat<3, 3> inv;
        if (n < 10 || !invert(A, inv)) return false;
        const Mat<3, 1> b = inv * aty;
        double ss_res = yy;  // y'y - 2 b'A'y + b'A'A b
        for (int i = 0; i < m; ++i) {
            beta[i] = b(i, 0);
            ss_res -= 2.0 * beta[i] * aty(i, 0);
            for (int j = 0; j < m; ++j) ss_res += beta[i] * ata(i, j) * b(j, 0);
        }
        const double ss_tot = yy - ysum * ysum / n;
        *r2 = ss_tot > 0 ? 1.0 - ss_res / ss_tot : 0.0;
        return true;
    }
};

int prepare(const SysIdSample* d, int n, int axis, float fc) {
    if (n > kSysIdMaxSamples) n = kSysIdMaxSamples;
    float t = 0.0f;
    for (int i = 0; i < n; ++i) {
        t += d[i].dt;
        g_t[i] = t;
        g_x[0][i] = d[i].theta[axis];
        g_x[1][i] = d[i].rate[axis];
        g_x[2][i] = d[i].wheel_angle[axis];
        g_x[3][i] = d[i].u[axis];
    }
    // Wheel angle -> wheel speed by central differences (zero phase). The
    // original value of sample i-1 is kept in `prev` because the array is
    // overwritten in place.
    float prev = g_x[2][0];
    for (int i = 0; i < n; ++i) {
        const int i0 = i > 0 ? i - 1 : 0;
        const int i1 = i < n - 1 ? i + 1 : n - 1;
        const float span = g_t[i1] - g_t[i0];
        const float speed = span > 0.0f ? (g_x[2][i1] - prev) / span : 0.0f;
        prev = g_x[2][i];
        g_x[2][i] = speed;
    }
    for (int k = 0; k < 4; ++k) filtfilt(g_x[k], n, g_t, fc);
    return n;
}

}  // namespace

SysIdResult identify_axis(const SysIdSample* data, int n, int naxes, float filter_hz, float kv_known) {
    SysIdResult r;
    if (!data || n < 50 || naxes < 1 || naxes > 2) {
        r.error = "not enough data";
        return r;
    }
    const bool kv_free = kv_known < 0.0f;
    LeastSquares body, wheel;
    body.m = wheel.m = kv_free ? 3 : 2;
    constexpr int kEdge = 10;  // skip filter start/end transients

    for (int ax = 0; ax < naxes; ++ax) {
        const int m = prepare(data, n, ax, filter_hz);
        for (int i = kEdge; i < m - kEdge; ++i) {
            const double dt2 = g_t[i + 1] - g_t[i - 1];
            if (!(dt2 > 0)) continue;
            const double th_dd = static_cast<double>(g_x[1][i + 1] - g_x[1][i - 1]) / dt2;
            const double s_d = static_cast<double>(g_x[2][i + 1] - g_x[2][i - 1]) / dt2;
            double phi[3];
            phi[0] = g_x[0][i];
            if (kv_free) {
                phi[1] = g_x[3][i];
                phi[2] = g_x[2][i];
            } else {
                phi[1] = g_x[3][i] - kv_known * g_x[2][i];
            }
            body.add(phi, th_dd);
            wheel.add(phi, s_d);
        }
    }
    r.samples = body.n;

    double bb[3] = {0, 0, 0}, bw[3] = {0, 0, 0}, r2b = 0, r2w = 0;
    if (!body.solve(bb, &r2b) || !wheel.solve(bw, &r2w)) {
        r.error = "regression singular (no excitation?)";
        return r;
    }
    r.a = static_cast<float>(bb[0]);
    r.b = static_cast<float>(bb[1]);
    const double bc = bw[1];
    r.c = static_cast<float>(bc - bb[1]);
    if (kv_free) {
        const double kv_b = bb[1] != 0 ? -bb[2] / bb[1] : 0;
        const double kv_w = bc != 0 ? -bw[2] / bc : 0;
        r.kv = static_cast<float>(0.5 * (kv_b + kv_w));
    } else {
        r.kv = kv_known;
    }
    r.r2_body = static_cast<float>(r2b);
    r.r2_wheel = static_cast<float>(r2w);

    if (!(r.a > 1.0f)) r.error = "gravity term a <= 1 (wrong balance point or axis?)";
    else if (!(r.b > 0.0f)) r.error = "motor term b <= 0 (axis sign wrong? run calib axes)";
    else if (!(r.c > 0.0f)) r.error = "wheel term c <= 0";
    else if (r.r2_body < 0.5f) r.error = "poor fit (R2 < 0.5): more excitation or less noise needed";
    else if (kv_free && !(r.kv >= 0.0f)) r.error = "negative back-EMF estimate";
    else r.ok = true;
    return r;
}

MotorIdResult identify_motor(const float* s, const float* u, int n, float dt, float filter_hz) {
    MotorIdResult r;
    if (!s || !u || n < 50 || !(dt > 0)) {
        r.error = "not enough data";
        return r;
    }
    if (n > kSysIdMaxSamples) n = kSysIdMaxSamples;
    for (int i = 0; i < n; ++i) {
        g_t[i] = static_cast<float>(i) * dt;
        g_x[2][i] = s[i];
        g_x[3][i] = u[i];
    }
    filtfilt(g_x[2], n, g_t, filter_hz);
    filtfilt(g_x[3], n, g_t, filter_hz);
    LeastSquares ls;
    ls.m = 2;
    for (int i = 10; i < n - 10; ++i) {
        const double s_d = static_cast<double>(g_x[2][i + 1] - g_x[2][i - 1]) / (2.0 * static_cast<double>(dt));
        const double phi[2] = {g_x[3][i], g_x[2][i]};
        ls.add(phi, s_d);
    }
    double beta[3] = {0, 0, 0}, r2 = 0;
    if (!ls.solve(beta, &r2)) {
        r.error = "regression singular";
        return r;
    }
    r.c = static_cast<float>(beta[0]);
    r.kv = beta[0] != 0 ? static_cast<float>(-beta[1] / beta[0]) : 0.0f;
    r.r2 = static_cast<float>(r2);
    if (!(r.c > 0.0f)) r.error = "c <= 0: positive command does not speed the wheel up (enc_invert?)";
    else if (!(r.kv >= 0.0f)) r.error = "negative back-EMF estimate";
    else if (r.r2 < 0.8f) r.error = "poor fit (R2 < 0.8)";
    else r.ok = true;
    return r;
}

}  // namespace cubli
