#include "wheel_speed.h"

#include "math3.h"

namespace cubli {

void WheelSpeedEstimator::reset(int32_t count, uint32_t t_us) {
    head_ = 0;
    filled_ = 1;
    counts_[0] = count;
    times_[0] = t_us;
    origin_ = last_ = count;
    raw_ = speed_ = 0.0f;
    last_t_ = t_us;
    lp_.reset();
}

float WheelSpeedEstimator::update(int32_t count, uint32_t t_us, int window, float cutoff_hz,
                                  int counts_per_rev) {
    if (window < 1) window = 1;
    if (window > kMaxWindow) window = kMaxWindow;
    const float dt = static_cast<float>(t_us - last_t_) * 1e-6f;
    last_t_ = t_us;
    last_ = count;

    head_ = (head_ + 1) % (kMaxWindow + 1);
    counts_[head_] = count;
    times_[head_] = t_us;
    if (filled_ < kMaxWindow + 1) ++filled_;

    const int span = (window < filled_ - 1) ? window : filled_ - 1;
    if (span <= 0 || counts_per_rev <= 0) return speed_;
    const int old = (head_ - span + (kMaxWindow + 1)) % (kMaxWindow + 1);
    const float t_span = static_cast<float>(times_[head_] - times_[old]) * 1e-6f;
    if (t_span <= 0.0f) return speed_;
    raw_ = static_cast<float>(count - counts_[old]) * (2.0f * kPi / static_cast<float>(counts_per_rev)) / t_span;
    speed_ = lp_.update(raw_, cutoff_hz, dt > 0.0f ? dt : t_span / static_cast<float>(span));
    return speed_;
}

float WheelSpeedEstimator::angle(int counts_per_rev) const {
    return counts_per_rev > 0
               ? static_cast<float>(last_ - origin_) * (2.0f * kPi / static_cast<float>(counts_per_rev))
               : 0.0f;
}

}  // namespace cubli
