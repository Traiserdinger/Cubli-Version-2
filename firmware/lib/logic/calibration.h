// Calibration data (motor axes + balance points) and the procedures that create it.
#pragma once

#include <cstddef>
#include <cstdint>

#include "balance.h"
#include "math3.h"

namespace cubli {

enum class PointKind : uint8_t { kUnknown = 0, kEdge = 1, kVertex = 2 };

struct BalancePoint {
    Vec3 up;            // world up in body frame at the balance point
    uint8_t valid = 0;
    PointKind kind = PointKind::kUnknown;
    int8_t motor = -1;  // edge: balancing motor
    uint8_t reserved = 0;
};

inline constexpr int kMaxBalancePoints = 8;  // 12 edges + 8 vertices exist; 8 slots is plenty

struct CalibrationData {
    Vec3 axis[kMotors] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    uint8_t axes_valid = 0;
    uint8_t reserved[3] = {0, 0, 0};
    BalancePoint points[kMaxBalancePoints];

    MotorAxes motor_axes() const;
    int point_count() const;
};

/// Edge: one motor axis is (nearly) horizontal -> that motor balances.
/// Vertex: all three axes make ~54.7 deg with the vertical.
PointKind classify_point(const MotorAxes& axes, Vec3 up, int* motor);

/// Nearest valid point to `up`, honouring a mode filter (0 any, 1 edge, 2 vertex).
/// Returns -1 if none. *angle is the angle to that point (rad).
int find_nearest_point(const CalibrationData& c, Vec3 up, int mode_filter, float* angle);

/// Stores a point; replaces an existing point within merge_rad. Returns slot or -1 if full.
int store_point(CalibrationData& c, Vec3 up, PointKind kind, int motor, float merge_rad);

/// Re-classify all points after the axes changed.
void reclassify_points(CalibrationData& c);

inline constexpr uint32_t kCalibBlobMagic = 0x43434132;  // "CCA2"
size_t calib_blob_size();
size_t calib_serialize(const CalibrationData& c, uint8_t* out, size_t cap);
bool calib_deserialize(CalibrationData& c, const uint8_t* in, size_t len);

/// Motor-axis calibration with the cube held loosely in the hand: each motor gets
/// a short positive pulse followed by an equal negative pulse; the change of the
/// body angular velocity during the positive pulse gives the axis direction.
class AxisCalibrator {
public:
    struct Config {
        float settle_s = 0.6f;
        float pulse_s = 0.12f;
        float pulse_u = 0.35f;
        float min_delta_rads = 0.15f;  // required body-rate change
    };
    enum class Status { kIdle, kRunning, kDone, kFailed };

    void start(const Config& cfg);
    void abort() { status_ = Status::kFailed; error_ = "aborted"; }
    /// Advances the sequence. Returns the command to apply to each motor.
    void step(float dt, Vec3 gyro, float u_out[kMotors]);

    Status status() const { return status_; }
    const char* error() const { return error_; }
    int current_motor() const { return motor_; }
    const MotorAxes& result() const { return result_; }
    float delta_magnitude(int k) const { return mag_[k]; }

private:
    enum class Phase { kSettle, kPositive, kNegative } phase_ = Phase::kSettle;
    Config cfg_;
    Status status_ = Status::kIdle;
    const char* error_ = "";
    int motor_ = 0;
    float t_ = 0.0f;
    Vec3 base_sum_, end_sum_;
    int base_n_ = 0, end_n_ = 0;
    float mag_[kMotors] = {0, 0, 0};
    MotorAxes result_;
};

}  // namespace cubli
