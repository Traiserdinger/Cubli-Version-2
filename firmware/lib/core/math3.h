// Small 3-D vector / quaternion / 3x3 matrix helpers (float, header-only).
#pragma once

#include <cmath>

namespace cubli {

inline constexpr float kPi = 3.14159265358979f;
inline constexpr float kDegToRad = kPi / 180.0f;
inline constexpr float kRadToDeg = 180.0f / kPi;

struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float ax, float ay, float az) : x(ax), y(ay), z(az) {}

    float& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
    float operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator-(Vec3 a) { return {-a.x, -a.y, -a.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(float s, Vec3 a) { return a * s; }
inline Vec3& operator+=(Vec3& a, Vec3 b) { a = a + b; return a; }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float norm(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalized(Vec3 a) {
    const float n = norm(a);
    return n > 1e-9f ? a * (1.0f / n) : Vec3{0.0f, 0.0f, 0.0f};
}
inline bool is_finite(Vec3 a) { return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z); }

/// Angle between two vectors (rad), robust near 0 and pi.
inline float angle_between(Vec3 a, Vec3 b) {
    return std::atan2(norm(cross(a, b)), dot(a, b));
}

/// Unit quaternion, scalar first. Rotates body-frame vectors into the world frame
/// (same convention as the IMU PCB EKF, context.md §8).
struct Quat {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
};

/// World "up" (+Z) expressed in the body frame = third row of R(q).
inline Vec3 up_in_body(const Quat& q) {
    return {2.0f * (q.x * q.z - q.w * q.y), 2.0f * (q.y * q.z + q.w * q.x),
            1.0f - 2.0f * (q.x * q.x + q.y * q.y)};
}

/// 3x3 matrix, row-major.
struct Mat3 {
    float m[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    static Mat3 from_columns(Vec3 c0, Vec3 c1, Vec3 c2) {
        Mat3 r;
        for (int i = 0; i < 3; ++i) {
            r.m[i][0] = c0[i];
            r.m[i][1] = c1[i];
            r.m[i][2] = c2[i];
        }
        return r;
    }
    Vec3 operator*(Vec3 v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }
    float det() const {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
               m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }
    /// Inverse; returns false (and leaves out unchanged) if near-singular.
    bool inverse(Mat3& out) const {
        const float d = det();
        if (!(std::fabs(d) > 1e-6f)) return false;
        const float inv = 1.0f / d;
        Mat3 r;
        r.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv;
        r.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv;
        r.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv;
        r.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv;
        r.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv;
        r.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv;
        r.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv;
        r.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv;
        r.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv;
        out = r;
        return true;
    }
};

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

}  // namespace cubli
