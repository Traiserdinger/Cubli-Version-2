// CubliV2 board pin map.
//
// Source of truth: context.md §4 (verified against docs/CubliV2_Schematic.pdf,
// sheet MCU, 2026-11-12). Do not change a pin here without changing context.md.
// Motor numbering is the SCHEMATIC numbering (net names), see context.md §6:
//   Motor 1 = J2 = silkscreen "MOTOR2" (top edge)
//   Motor 2 = J3 = silkscreen "MOTOR3" (left edge)
//   Motor 3 = J4 = silkscreen "MOTOR1" (right edge)
#pragma once

#include <cstdint>

namespace board {

inline constexpr int kNumMotors = 3;

inline constexpr int kPinStatusLed = 2;   // D1 via R18 220R to GND, active HIGH. Strapping pin.
inline constexpr int kPinBootButton = 0;  // SW3, active LOW, R4 10k pull-up. Shared with auto-boot (Q1).
inline constexpr int kPinBrake = 18;      // Shared START/BRAKE, R32 10k pull-down. LOW = brake all.
inline constexpr int kPinBuzzer = 19;     // Q3 MMBT3904 low-side, passive buzzer from +5 V. R37 pull-down.
inline constexpr int kPinI2cSda = 21;     // R9 4k7 pull-up. INA219 @ 0x40.
inline constexpr int kPinI2cScl = 22;     // R17 4k7 pull-up.
inline constexpr int kPinImuRx = 16;      // UART2 RX  <- J8 pin 2 (SENS_RX) <- IMU PCB TX
inline constexpr int kPinImuTx = 17;      // UART2 TX  -> J8 pin 1 (SENS_TX) -> IMU PCB RX

//                                    Motor1 Motor2 Motor3
inline constexpr int kPinPwm[kNumMotors] = {25, 26, 27};  // PWMx, R36/R38/R39 10k pull-down
inline constexpr int kPinDir[kNumMotors] = {4, 13, 14};   // ROTATIONx, R33/R34/R35 10k pull-down
inline constexpr int kPinEncA[kNumMotors] = {34, 36, 32}; // ENC_Ax via 74LVC14 (push-pull)
inline constexpr int kPinEncB[kNumMotors] = {35, 39, 33}; // ENC_Bx via 74LVC14 (push-pull)

inline constexpr uint8_t kIna219Address = 0x40;  // A0 = A1 = GND (context.md §8)

// Human-readable connector mapping for bench-test output (context.md §6).
inline constexpr const char* kMotorConnector[kNumMotors] = {"J2", "J3", "J4"};
inline constexpr const char* kMotorSilkscreen[kNumMotors] = {"MOTOR2", "MOTOR3", "MOTOR1"};
inline constexpr const char* kMotorBoardEdge[kNumMotors] = {"top", "left", "right"};

// Pins that must never be configured: GPIO6-11 (module flash), GPIO12/15
// (strapping, not connected), GPIO5/23 (not connected).
constexpr bool is_forbidden_pin(int gpio) {
    return (gpio >= 6 && gpio <= 12) || gpio == 15;
}

namespace detail {
constexpr bool all_allowed() {
    for (int i = 0; i < kNumMotors; ++i) {
        if (is_forbidden_pin(kPinPwm[i]) || is_forbidden_pin(kPinDir[i]) ||
            is_forbidden_pin(kPinEncA[i]) || is_forbidden_pin(kPinEncB[i])) {
            return false;
        }
    }
    return !is_forbidden_pin(kPinBrake) && !is_forbidden_pin(kPinBuzzer) &&
           !is_forbidden_pin(kPinStatusLed) && !is_forbidden_pin(kPinImuRx) &&
           !is_forbidden_pin(kPinImuTx) && !is_forbidden_pin(kPinI2cSda) &&
           !is_forbidden_pin(kPinI2cScl);
}
constexpr bool outputs_are_output_capable() {
    // GPIO34-39 are input-only on the ESP32.
    for (int i = 0; i < kNumMotors; ++i) {
        if (kPinPwm[i] >= 34 || kPinDir[i] >= 34) return false;
    }
    return kPinBrake < 34 && kPinBuzzer < 34 && kPinStatusLed < 34 && kPinImuTx < 34;
}
}  // namespace detail

static_assert(detail::all_allowed(), "pin map uses a flash/strapping pin");
static_assert(detail::outputs_are_output_capable(), "output assigned to an input-only GPIO");

}  // namespace board
