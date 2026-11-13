#pragma once

#include <cmath>
#include <cstddef>

namespace cubli {

/// First-order low-pass filter, y += alpha (x - y), alpha from cutoff and dt.
class LowPass1 {
public:
    void reset(float y = 0.0f) { y_ = y; primed_ = false; }
    float update(float x, float cutoff_hz, float dt) {
        if (!primed_) {
            y_ = x;
            primed_ = true;
            return y_;
        }
        if (cutoff_hz <= 0.0f) {
            y_ = x;
            return y_;
        }
        const float rc = 1.0f / (2.0f * 3.14159265f * cutoff_hz);
        y_ += (dt / (rc + dt)) * (x - y_);
        return y_;
    }
    float value() const { return y_; }

private:
    float y_ = 0.0f;
    bool primed_ = false;
};

/// Moving average over the last N samples (N fixed at compile time).
template <size_t N>
class MovingAverage {
public:
    float update(float x) {
        if (count_ < N) {
            ++count_;
        } else {
            sum_ -= static_cast<double>(buf_[idx_]);
        }
        buf_[idx_] = x;
        sum_ += static_cast<double>(x);
        idx_ = (idx_ + 1) % N;
        return value();
    }
    float value() const { return count_ ? static_cast<float>(sum_ / static_cast<double>(count_)) : 0.0f; }
    bool full() const { return count_ == N; }
    void reset() { count_ = idx_ = 0; sum_ = 0.0; }

private:
    float buf_[N] = {};
    size_t count_ = 0, idx_ = 0;
    double sum_ = 0.0;
};

/// True once `condition` has held continuously for `hold_s` seconds.
class Persistence {
public:
    bool update(bool condition, float dt, float hold_s) {
        t_ = condition ? t_ + dt : 0.0f;
        return condition && t_ >= hold_s;
    }
    void reset() { t_ = 0.0f; }

private:
    float t_ = 0.0f;
};

}  // namespace cubli
