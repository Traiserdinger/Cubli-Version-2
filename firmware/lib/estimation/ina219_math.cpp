#include "ina219_math.h"

namespace cubli::ina219 {

float pga_range_v(uint8_t pga) {
    static constexpr float kRange[4] = {0.040f, 0.080f, 0.160f, 0.320f};
    return kRange[pga & 3u];
}

uint8_t select_pga(float max_current_a, float shunt_ohm) {
    const float v = max_current_a * shunt_ohm;
    for (uint8_t pga = 0; pga < 3; ++pga) {
        if (v <= pga_range_v(pga)) return pga;
    }
    return 3;
}

uint16_t make_config(uint8_t pga) {
    constexpr uint16_t kBrng16V = 0u << 13;
    constexpr uint16_t kAdc12bitX8 = 0xB;  // 1011b: 8 samples, 4.26 ms
    constexpr uint16_t kModeContinuous = 0x7;
    return static_cast<uint16_t>(kBrng16V | ((pga & 3u) << 11) | (kAdc12bitX8 << 7) | (kAdc12bitX8 << 3) |
                                 kModeContinuous);
}

BusReading decode_bus(uint16_t reg) {
    BusReading r;
    r.volts = static_cast<float>(reg >> 3) * kBusLsbV;
    r.conversion_ready = (reg & 0x2u) != 0;
    r.overflow = (reg & 0x1u) != 0;
    return r;
}

float decode_current(uint16_t shunt_reg, float shunt_ohm) {
    if (!(shunt_ohm > 0.0f)) return 0.0f;
    const int16_t raw = static_cast<int16_t>(shunt_reg);
    return static_cast<float>(raw) * kShuntLsbV / shunt_ohm;
}

}  // namespace cubli::ina219
