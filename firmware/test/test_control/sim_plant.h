// Physical simulation of the cube used to test the controller and the system
// identification on the PC. Independent of the controller's linear model:
// nonlinear gravity, wheel gyroscopic coupling (vertex), saturating motors with
// back-EMF and friction, encoder quantisation, sensor noise, 1-sample delay and
// an IMU mounted at an arbitrary rotation.
#pragma once
#pragma GCC diagnostic ignored "-Wdouble-promotion"

#include <cmath>
#include <cstdint>
#include <random>

#include "math3.h"

namespace sim {

using cubli::Vec3;

struct CubeParams {
    double side = 0.15;      // m
    double mass = 1.2;       // kg
    double wheel_I = 7.2e-4; // kg m^2 (axial)
    double km = 0.12;        // N m per unit command
    double kv = 0.0025;      // command per rad/s (no-load speed 400 rad/s)
    double friction = 2e-5;  // N m per rad/s
    double gyro_noise = 0.004;   // rad/s rms
    double up_noise = 0.0005;    // rad rms
    double g = 9.81;
};

inline Vec3 rot(const double R[3][3], Vec3 v) {
    return {static_cast<float>(R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z),
            static_cast<float>(R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z),
            static_cast<float>(R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z)};
}

/// Rodrigues rotation matrix.
inline void axis_angle(const double axis_in[3], double ang, double R[3][3]) {
    double n = std::sqrt(axis_in[0] * axis_in[0] + axis_in[1] * axis_in[1] + axis_in[2] * axis_in[2]);
    const double x = axis_in[0] / n, y = axis_in[1] / n, z = axis_in[2] / n;
    const double c = std::cos(ang), s = std::sin(ang), C = 1 - c;
    R[0][0] = c + x * x * C;     R[0][1] = x * y * C - z * s; R[0][2] = x * z * C + y * s;
    R[1][0] = y * x * C + z * s; R[1][1] = c + y * y * C;     R[1][2] = y * z * C - x * s;
    R[2][0] = z * x * C - y * s; R[2][1] = z * y * C + x * s; R[2][2] = c + z * z * C;
}

/// Common sensor/actuator plumbing. Cube frame: motor k spins about cube axis k.
struct PlantBase {
    CubeParams p;
    double mount[3][3];             // IMU = mount * cube
    double wheel_sign[3] = {1, -1, 1};  // wheel axis w_k = sign_k * e_k (cube frame)
    double s[3] = {0, 0, 0};        // wheel speeds relative to body
    double phi[3] = {0, 0, 0};      // wheel angles
    double u_applied[3] = {0, 0, 0};
    std::mt19937 rng{12345};
    std::normal_distribution<double> nd{0.0, 1.0};

    PlantBase() {
        const double ax[3] = {0.3, -0.5, 0.8};
        axis_angle(ax, 0.9, mount);  // arbitrary IMU mounting
    }
    double motor_torque(int k) const {
        double u = u_applied[k];
        if (u > 1) u = 1;
        if (u < -1) u = -1;
        return p.km * (u - p.kv * s[k]) - p.friction * s[k];
    }
    /// True body-acceleration direction for +u on motor k, in IMU coordinates.
    Vec3 true_axis_imu(int k) const {
        Vec3 e{0, 0, 0};
        e[k] = static_cast<float>(-wheel_sign[k]);
        return rot(mount, e);
    }
    int32_t encoder(int k) const {
        return static_cast<int32_t>(std::floor(phi[k] * 400.0 / (2.0 * M_PI)));
    }
    float noise(double sigma) { return static_cast<float>(sigma * nd(rng)); }
};

/// Edge: rotation about cube axis 2 (motor 3) only.
struct EdgePlant : PlantBase {
    double theta = 0, theta_dot = 0;
    static constexpr int kMotor = 2;

    double J() const {
        const double s2 = p.side * p.side;
        return p.mass * s2 / 6.0 + p.mass * s2 / 2.0;
    }
    double l() const { return p.side / std::sqrt(2.0); }
    /// Model parameters the identification should recover.
    double a() const { return p.mass * p.g * l() / J(); }
    double b() const { return p.km / J(); }
    double c() const { return p.km / p.wheel_I; }

    Vec3 up_eq_cube() const { return {static_cast<float>(1 / std::sqrt(2.0)), static_cast<float>(1 / std::sqrt(2.0)), 0}; }

    void step(double dt) {
        const int sub = 10;
        const double h = dt / sub;
        const double w_dot_e = wheel_sign[kMotor];  // w . e_axis
        for (int i = 0; i < sub; ++i) {
            const double tau = motor_torque(kMotor);
            const double th_dd = (p.mass * p.g * l() * std::sin(theta) - tau * w_dot_e) / J();
            const double s_d = tau / p.wheel_I - th_dd * w_dot_e;
            theta_dot += th_dd * h;
            theta += theta_dot * h;
            s[kMotor] += s_d * h;
            phi[kMotor] += s[kMotor] * h;
            for (int k = 0; k < 3; ++k) {
                if (k == kMotor) continue;
                const double tk = motor_torque(k);
                s[k] += tk / p.wheel_I * h;
                phi[k] += s[k] * h;
            }
        }
    }
    // Body rotates about e = -wheel_sign * e_z ... we use the cube z axis as the
    // rotation axis; theta > 0 is a rotation about +e_z.
    Vec3 up_imu() {
        const double ez[3] = {0, 0, 1};
        double R[3][3];
        axis_angle(ez, -theta, R);
        const Vec3 up_c = rot(R, up_eq_cube());
        Vec3 up = rot(mount, up_c);
        up.x += noise(p.up_noise); up.y += noise(p.up_noise); up.z += noise(p.up_noise);
        return cubli::normalized(up);
    }
    Vec3 up_eq_imu() const { return rot(mount, up_eq_cube()); }
    Vec3 gyro_imu() {
        Vec3 g = rot(mount, Vec3{0, 0, static_cast<float>(theta_dot)});
        g.x += noise(p.gyro_noise); g.y += noise(p.gyro_noise); g.z += noise(p.gyro_noise);
        return g;
    }
};

/// Vertex: free rotation about the pivot vertex (spherical joint), full 3-D.
struct VertexPlant : PlantBase {
    double q[4] = {1, 0, 0, 0};  // cube -> world
    double w[3] = {0, 0, 0};     // body rate, cube frame
    double Jp[3][3];
    double Jinv[3][3];
    double rc[3];

    VertexPlant() {
        const double s2 = p.side * p.side;
        const double jc = p.mass * s2 / 6.0;
        for (int i = 0; i < 3; ++i) rc[i] = p.side / 2;
        const double r2 = rc[0] * rc[0] + rc[1] * rc[1] + rc[2] * rc[2];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) Jp[i][j] = (i == j ? jc + p.mass * r2 : 0.0) - p.mass * rc[i] * rc[j];
        invert3(Jp, Jinv);
    }
    static void invert3(const double m[3][3], double o[3][3]) {
        const double d = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                         m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                         m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        o[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / d;
        o[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / d;
        o[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / d;
        o[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) / d;
        o[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / d;
        o[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) / d;
        o[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / d;
        o[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / d;
        o[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / d;
    }
    /// Free body rotating about its centre of mass (cube held loosely in the hand).
    void make_free() {
        p.g = 0.0;
        const double jc = p.mass * p.side * p.side / 6.0;
        for (int i = 0; i < 3; ++i) {
            rc[i] = 0.0;
            for (int j = 0; j < 3; ++j) Jp[i][j] = i == j ? jc : 0.0;
        }
        invert3(Jp, Jinv);
    }
    double horiz_J() const {
        const double s2 = p.side * p.side;
        return p.mass * s2 / 6.0 + p.mass * 3.0 * s2 / 4.0;
    }
    double a() const { return p.mass * p.g * (p.side * std::sqrt(3.0) / 2) / horiz_J(); }
    double b() const { return p.km / horiz_J(); }
    double c() const { return p.km / p.wheel_I; }

    /// Upright with a small tilt (rad) about a horizontal axis.
    void set_upright(double tilt) {
        // Rotation taking rc_hat to world z.
        const double n = std::sqrt(3.0);
        const double r[3] = {1 / n, 1 / n, 1 / n};
        const double z[3] = {0, 0, 1};
        double ax[3] = {r[1] * z[2] - r[2] * z[1], r[2] * z[0] - r[0] * z[2], r[0] * z[1] - r[1] * z[0]};
        const double ang = std::acos(r[2]);
        const double an = std::sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]);
        for (double& v : ax) v /= an;
        double hq[4] = {std::cos(ang / 2), ax[0] * std::sin(ang / 2), ax[1] * std::sin(ang / 2), ax[2] * std::sin(ang / 2)};
        // Extra tilt about world x.
        const double tq[4] = {std::cos(tilt / 2), std::sin(tilt / 2), 0, 0};
        quat_mul(tq, hq, q);
    }
    static void quat_mul(const double a[4], const double b[4], double o[4]) {
        o[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
        o[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
        o[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
        o[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    }
    void up_cube(double u[3]) const {
        u[0] = 2 * (q[1] * q[3] - q[0] * q[2]);
        u[1] = 2 * (q[2] * q[3] + q[0] * q[1]);
        u[2] = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    }
    void step(double dt) {
        const int sub = 10;
        const double h = dt / sub;
        for (int it = 0; it < sub; ++it) {
            double up[3];
            up_cube(up);
            // gravity torque r x (-m g up)
            const double F[3] = {-p.mass * p.g * up[0], -p.mass * p.g * up[1], -p.mass * p.g * up[2]};
            double M[3] = {rc[1] * F[2] - rc[2] * F[1], rc[2] * F[0] - rc[0] * F[2], rc[0] * F[1] - rc[1] * F[0]};
            double H[3];
            for (int i = 0; i < 3; ++i) H[i] = Jp[i][0] * w[0] + Jp[i][1] * w[1] + Jp[i][2] * w[2];
            double tau[3];
            for (int k = 0; k < 3; ++k) {
                tau[k] = motor_torque(k);
                H[k] += p.wheel_I * (s[k] + w[k] * wheel_sign[k]) * wheel_sign[k];
                M[k] -= tau[k] * wheel_sign[k];
            }
            const double wxH[3] = {w[1] * H[2] - w[2] * H[1], w[2] * H[0] - w[0] * H[2], w[0] * H[1] - w[1] * H[0]};
            double rhs[3], wd[3];
            for (int i = 0; i < 3; ++i) rhs[i] = M[i] - wxH[i];
            for (int i = 0; i < 3; ++i) wd[i] = Jinv[i][0] * rhs[0] + Jinv[i][1] * rhs[1] + Jinv[i][2] * rhs[2];
            for (int k = 0; k < 3; ++k) {
                const double sd = tau[k] / p.wheel_I - wd[k] * wheel_sign[k];
                s[k] += sd * h;
                phi[k] += s[k] * h;
            }
            for (int i = 0; i < 3; ++i) w[i] += wd[i] * h;
            const double dq[4] = {0, w[0], w[1], w[2]};
            double qd[4];
            quat_mul(q, dq, qd);
            for (int i = 0; i < 4; ++i) q[i] += 0.5 * qd[i] * h;
            const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            for (double& v : q) v /= n;
        }
    }
    Vec3 up_imu() {
        double u[3];
        up_cube(u);
        Vec3 up = rot(mount, Vec3{static_cast<float>(u[0]), static_cast<float>(u[1]), static_cast<float>(u[2])});
        up.x += noise(p.up_noise); up.y += noise(p.up_noise); up.z += noise(p.up_noise);
        return cubli::normalized(up);
    }
    Vec3 up_eq_imu() const {
        const float v = static_cast<float>(1 / std::sqrt(3.0));
        return rot(mount, Vec3{v, v, v});
    }
    Vec3 gyro_imu() {
        Vec3 g = rot(mount, Vec3{static_cast<float>(w[0]), static_cast<float>(w[1]), static_cast<float>(w[2])});
        g.x += noise(p.gyro_noise); g.y += noise(p.gyro_noise); g.z += noise(p.gyro_noise);
        return g;
    }
    double tilt() const {
        double u[3];
        up_cube(u);
        const double n = std::sqrt(3.0);
        return std::acos(std::fmin(1.0, (u[0] + u[1] + u[2]) / n));
    }
};

}  // namespace sim
