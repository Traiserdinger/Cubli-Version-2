// Fixed hardware constants (not user-tunable). Tunable values live in
// lib/logic/params.h. Every constant cites its source.
#pragma once

#include <cstdint>

namespace hw {

// ---- Motor PWM (context.md §5.2, §9) ----
inline constexpr uint32_t kPwmFrequencyHz = 20000;  // Nidec 24H starts reliably 16-26 kHz
inline constexpr uint32_t kPwmResolutionBits = 10;  // 80 MHz / 20 kHz = 4000 -> <= 11 bits
inline constexpr uint32_t kPwmDutyMax = 1u << kPwmResolutionBits;  // duty == max -> constant HIGH = stop
static_assert(kPwmFrequencyHz * kPwmDutyMax <= 80000000u, "LEDC frequency/resolution not reachable");

// ---- Encoder (context.md §5.2, §7) ----
inline constexpr int kEncoderCountsPerRev = 400;  // 100 lines, x4 quadrature (verify: wiretest)
inline constexpr uint32_t kEncoderGlitchNs = 1000; // RC on board is ~1 us; PCNT filter <= 12.7 us

// ---- Buzzer (context.md §8: AT-0927-TT-2-R, passive) ----
inline constexpr uint32_t kBuzzerDefaultHz = 2700;

// ---- INA219 (context.md §8) ----
inline constexpr uint32_t kI2cClockHz = 400000;

// ---- IMU PCB UART (context.md §8 "IMU PCB") ----
inline constexpr uint32_t kImuBootBaud = 57600;   // IMU PCB baud at power-up
inline constexpr uint32_t kImuFastBaud = 460800;  // frame v2 baud (ADuM1201ARZ <= 1 Mbps)

// ---- Console ----
inline constexpr uint32_t kConsoleBaud = 921600;  // CH340C supports up to 2 Mbps

// ---- 3S battery (context.md §3) ----
inline constexpr float kBatteryPresentMinV = 5.0f;  // below: running from USB, no pack
inline constexpr float kBatteryFullV = 12.6f;
inline constexpr float kBatteryAbsoluteMinV = 9.0f;
inline constexpr float kFuseLimitA = 5.0f;  // 2x 2.5 A PTC in parallel

}  // namespace hw
