// INA219 register arithmetic (datasheet SBOS448G). Kept free of I/O so it can be
// unit-tested. The shunt value is a runtime parameter (context.md §8: 50 mOhm
// fitted, 10 mOhm on the schematic).
#pragma once

#include <cstdint>

namespace cubli::ina219 {

inline constexpr uint8_t kRegConfig = 0x00;
inline constexpr uint8_t kRegShunt = 0x01;
inline constexpr uint8_t kRegBus = 0x02;
inline constexpr uint16_t kConfigReset = 0x8000;
inline constexpr float kShuntLsbV = 10e-6f;  // 10 uV
inline constexpr float kBusLsbV = 4e-3f;     // 4 mV

/// Smallest PGA range (40/80/160/320 mV) that covers max_current_a * shunt_ohm.
/// Returns the PG field value 0..3.
uint8_t select_pga(float max_current_a, float shunt_ohm);
float pga_range_v(uint8_t pga);

/// Config: 16 V bus range, given PGA, 12-bit x8 averaging on both ADCs,
/// continuous shunt + bus conversion.
uint16_t make_config(uint8_t pga);

struct BusReading {
    float volts = 0.0f;
    bool conversion_ready = false;
    bool overflow = false;
};
BusReading decode_bus(uint16_t reg);

/// Current in amps from the signed shunt register.
float decode_current(uint16_t shunt_reg, float shunt_ohm);

}  // namespace cubli::ina219
