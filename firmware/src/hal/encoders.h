// Quadrature encoders on the PCNT peripheral (x4 decoding, glitch filter, no
// GPIO interrupts). Counts are accumulated beyond the 16-bit hardware range.
#pragma once

#include <cstdint>

#include "board_pins.h"

namespace hal {

class Encoders {
public:
    bool init();
    /// Raw accumulated count (no inversion applied).
    int32_t count(int motor) const;
    bool ok() const { return ok_; }

private:
    void* unit_[board::kNumMotors] = {nullptr, nullptr, nullptr};
    bool ok_ = false;
};

}  // namespace hal
