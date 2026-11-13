#include "calibration.h"

#include <cmath>
#include <cstring>

#include "crc.h"

namespace cubli {

MotorAxes CalibrationData::motor_axes() const {
    MotorAxes a;
    for (int k = 0; k < kMotors; ++k) a.axis[k] = axis[k];
    if (axes_valid) a.finalize();
    return a;
}

int CalibrationData::point_count() const {
    int n = 0;
    for (const auto& p : points) n += p.valid ? 1 : 0;
    return n;
}

PointKind classify_point(const MotorAxes& axes, Vec3 up, int* motor) {
    if (motor) *motor = -1;
    if (!axes.valid) return PointKind::kUnknown;
    up = normalized(up);
    float c[kMotors];
    int kmin = 0;
    for (int k = 0; k < kMotors; ++k) {
        c[k] = std::fabs(dot(axes.axis[k], up));
        if (c[k] < c[kmin]) kmin = k;
    }
    // Edge: up = (n_i + n_j)/sqrt2 -> cosines 0, .707, .707
    // (a face has two horizontal axes and one vertical: neither edge nor vertex)
    bool edge = c[kmin] < 0.26f;  // one axis within 15 deg of horizontal
    for (int k = 0; k < kMotors; ++k) {
        if (k != kmin) edge = edge && c[k] > 0.5f && c[k] < 0.87f;
    }
    if (edge) {
        if (motor) *motor = kmin;
        return PointKind::kEdge;
    }
    // Vertex: cosines all 1/sqrt3 = .577
    bool vertex = true;
    for (int k = 0; k < kMotors; ++k) vertex = vertex && c[k] > 0.40f && c[k] < 0.75f;
    return vertex ? PointKind::kVertex : PointKind::kUnknown;
}

int find_nearest_point(const CalibrationData& c, Vec3 up, int mode_filter, float* angle) {
    int best = -1;
    float best_ang = 1e9f;
    for (int i = 0; i < kMaxBalancePoints; ++i) {
        const BalancePoint& p = c.points[i];
        if (!p.valid) continue;
        if (mode_filter == 1 && p.kind != PointKind::kEdge) continue;
        if (mode_filter == 2 && p.kind != PointKind::kVertex) continue;
        const float a = angle_between(up, p.up);
        if (a < best_ang) {
            best_ang = a;
            best = i;
        }
    }
    if (angle) *angle = best_ang;
    return best;
}

int store_point(CalibrationData& c, Vec3 up, PointKind kind, int motor, float merge_rad) {
    int slot = -1;
    for (int i = 0; i < kMaxBalancePoints; ++i) {
        if (c.points[i].valid && angle_between(up, c.points[i].up) < merge_rad) {
            slot = i;
            break;
        }
    }
    for (int i = 0; slot < 0 && i < kMaxBalancePoints; ++i)
        if (!c.points[i].valid) slot = i;
    if (slot < 0) return -1;
    BalancePoint& p = c.points[slot];
    p.up = normalized(up);
    p.valid = 1;
    p.kind = kind;
    p.motor = static_cast<int8_t>(motor);
    return slot;
}

void reclassify_points(CalibrationData& c) {
    const MotorAxes axes = c.motor_axes();
    for (auto& p : c.points) {
        if (!p.valid) continue;
        int m = -1;
        p.kind = classify_point(axes, p.up, &m);
        p.motor = static_cast<int8_t>(m);
    }
}

namespace {
struct CalibHeader {
    uint32_t magic;
    uint16_t size;
    uint16_t crc;
};
}  // namespace

size_t calib_blob_size() { return sizeof(CalibHeader) + sizeof(CalibrationData); }

size_t calib_serialize(const CalibrationData& c, uint8_t* out, size_t cap) {
    if (cap < calib_blob_size()) return 0;
    std::memcpy(out + sizeof(CalibHeader), &c, sizeof(c));
    const CalibHeader h{kCalibBlobMagic, static_cast<uint16_t>(sizeof(c)),
                        crc16_ccitt(out + sizeof(CalibHeader), sizeof(c))};
    std::memcpy(out, &h, sizeof(h));
    return calib_blob_size();
}

bool calib_deserialize(CalibrationData& c, const uint8_t* in, size_t len) {
    if (!in || len < calib_blob_size()) return false;
    CalibHeader h;
    std::memcpy(&h, in, sizeof(h));
    if (h.magic != kCalibBlobMagic || h.size != sizeof(CalibrationData)) return false;
    if (crc16_ccitt(in + sizeof(CalibHeader), sizeof(CalibrationData)) != h.crc) return false;
    std::memcpy(&c, in + sizeof(CalibHeader), sizeof(c));
    return true;
}

// ---------------------------------------------------------------------------

void AxisCalibrator::start(const Config& cfg) {
    cfg_ = cfg;
    status_ = Status::kRunning;
    error_ = "";
    motor_ = 0;
    phase_ = Phase::kSettle;
    t_ = 0.0f;
    base_sum_ = end_sum_ = Vec3{};
    base_n_ = end_n_ = 0;
}

void AxisCalibrator::step(float dt, Vec3 gyro, float u_out[kMotors]) {
    for (int k = 0; k < kMotors; ++k) u_out[k] = 0.0f;
    if (status_ != Status::kRunning) return;
    t_ += dt;
    switch (phase_) {
        case Phase::kSettle:
            // Baseline: mean body rate over the last 0.2 s of the settle time.
            if (t_ > cfg_.settle_s - 0.2f) {
                base_sum_ += gyro;
                ++base_n_;
            }
            if (t_ >= cfg_.settle_s) {
                phase_ = Phase::kPositive;
                t_ = 0.0f;
            }
            break;
        case Phase::kPositive:
            u_out[motor_] = cfg_.pulse_u;
            if (t_ > cfg_.pulse_s - 0.02f) {
                end_sum_ += gyro;
                ++end_n_;
            }
            if (t_ >= cfg_.pulse_s) {
                const Vec3 base = base_n_ ? base_sum_ * (1.0f / base_n_) : Vec3{};
                const Vec3 end = end_n_ ? end_sum_ * (1.0f / end_n_) : Vec3{};
                const Vec3 d = end - base;
                mag_[motor_] = norm(d);
                result_.axis[motor_] = normalized(d);
                phase_ = Phase::kNegative;
                t_ = 0.0f;
            }
            break;
        case Phase::kNegative:
            u_out[motor_] = -cfg_.pulse_u;  // undo the wheel speed and body motion
            if (t_ >= cfg_.pulse_s) {
                if (mag_[motor_] < cfg_.min_delta_rads) {
                    status_ = Status::kFailed;
                    error_ = "body barely moved: hold the cube more loosely, check motor wiring";
                    return;
                }
                ++motor_;
                base_sum_ = end_sum_ = Vec3{};
                base_n_ = end_n_ = 0;
                phase_ = Phase::kSettle;
                t_ = 0.0f;
                if (motor_ >= kMotors) {
                    if (result_.finalize(0.35f)) {
                        status_ = Status::kDone;
                    } else {
                        status_ = Status::kFailed;
                        error_ = "axes not orthogonal (|cos| > 0.35): repeat, hold the cube still between pulses";
                    }
                }
            }
            break;
    }
}

}  // namespace cubli
