// Wheel speed from encoder counts.
//
// At 1 kHz a single count (400 CPR) is 15.7 rad/s, so a one-sample difference is
// useless. The estimator differentiates over a sliding window of W samples
// (resolution 2*pi/400/(W*dt)) and then low-pass filters the result.
#pragma once

#include <cstdint>

#include "filters.h"

namespace cubli {

class WheelSpeedEstimator {
public:
    static constexpr int kMaxWindow = 32;

    void reset(int32_t count, uint32_t t_us);
    /// count: accumulated encoder count (sign already corrected); t_us: sample
    /// time in microseconds (wrap-around safe). Returns the filtered wheel speed
    /// in rad/s (relative to the body).
    float update(int32_t count, uint32_t t_us, int window, float cutoff_hz, int counts_per_rev);

    float speed() const { return speed_; }
    float raw_speed() const { return raw_; }
    /// Wheel angle (rad) since reset().
    float angle(int counts_per_rev) const;

private:
    int32_t counts_[kMaxWindow + 1] = {};
    uint32_t times_[kMaxWindow + 1] = {};
    int head_ = 0, filled_ = 0;
    int32_t origin_ = 0, last_ = 0;
    float raw_ = 0.0f, speed_ = 0.0f;
    uint32_t last_t_ = 0;
    LowPass1 lp_;
};

}  // namespace cubli
