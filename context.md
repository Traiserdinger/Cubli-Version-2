# CubliV2 — Hardware Context for Firmware Development

> **Purpose of this file:** single source of truth about the CubliV2 hardware, extracted from the
> Altium schematic (`hardware/*.SchDoc`, exported as `docs/CubliV2_Schematic.pdf`) and the PCB
> (`hardware/CubliV2PCB.PcbDoc`). Firmware must follow this file. If code and this file disagree,
> this file wins; if this file and the schematic PDF disagree, **stop and ask the owner**.

- **Repository:** https://github.com/Traiserdinger/CubliV2
- **Owner:** Tuan (IUH — Industrial University of Ho Chi Minh City)
- **Project:** self-balancing reaction-wheel cube ("Cubli"), inspired by RemRC's ESP32 Cubli,
  redesigned PCB with better power, protection, sensing and signal conditioning.
- **Firmware location:** `firmware/` (PlatformIO project — to be created).
- **Status:** PCB designed (Altium Designer). Firmware: not started.

---

## 1. Repository layout

```
CubliV2/
├── README.md                 GitHub landing page
├── context.md                THIS FILE — hardware facts for firmware work
├── CLAUDE.md                 Claude Code entry point (imports context.md)
├── hardware/                 Altium project (open hardware/CubliV2.PrjPcb)
│   ├── CubliV2.PrjPcb        project
│   ├── MCU.SchDoc            sheet 1 — ESP32, USB-UART, Schmitt, buzzer, LEDs, buttons
│   ├── Power.SchDoc          sheet 2 — 2x buck, LDO, INA219, power switch
│   ├── Connector.SchDoc      sheet 3 (top level) — XT60, motor, IMU and 1S connectors
│   ├── CubliV2PCB.PcbDoc     2-layer PCB
│   └── Library/              component libraries (IntLib/SchLib/PcbLib/STEP)
├── manufacturing/            gerber/, drill/, bom/, reports/
├── docs/                     schematic PDF, images, Claude Code prompt
└── firmware/                 ESP32 firmware (PlatformIO)
```

Altium binaries (`.SchDoc`, `.PcbDoc`) cannot be parsed reliably as text — use the PDF and this file.

---

## 2. System overview

```
 3S LiPo (9.0–12.6 V) ─► XT60 (J6) ─► 2× PTC 2.5 A (F1‖F2) ─► +12 ─┬─ TVS SMBJ15A (D3)
                                                                   │
                                     INA219 (U7, 0x40) measures ◄──┤ R15 = 10 mΩ shunt
                                                                   ▼
                                                               +12_LOAD
            ┌───────────────────────────┬──────────────────────────┼──────────────────────┐
            ▼                           ▼                          ▼                      ▼
   3× Nidec 24H motors       TPS5430 (U6) → +5 V          TPS5430 (U3) → +4 V
   (J2, J3, J4 pin 1)        encoders, buzzer, SW1        J7 "BAT 1S" (1S-battery substitute
                                   │                       for an external sensor board)
                                   ▼
              USB-C VBUS ─► SW1 (MTS-103, ON-OFF-ON) ─► POWER ─► AMS1117-3.3 (U4) ─► +3.3_MCU
                                                                  ESP32, CH340C, 74LVC14, INA219, IMU PCB
```

- **MCU:** ESP32-WROOM-32E, 4 MB flash (U1). Dual core 240 MHz, no PSRAM.
- **USB:** USB-C (J1, CC1/CC2 5.1 kΩ to GND, sink only) → CH340C (U2) → UART0. Classic
  2× S8050 auto-reset/auto-boot circuit (DTR/RTS → EN/IO0) — `esptool` auto-programming works.
- **Motors:** 3× Nidec 24H brushless motor with built-in driver + 100-line encoder.
- **Sensors on board:** INA219 (battery voltage + total current). **No IMU on this board** — the IMU
  lives on a separate "IMU PCB" plugged into J8 and talks over UART2 (see §8).

---

## 3. Power tree (computed from schematic values)

| Rail | Source | Value | Calculation / note | Loads |
|---|---|---|---|---|
| +12 | Battery via F1‖F2 (2× MF-MSMF250/16X, 2.5 A hold each) | 9.0–12.6 V (3S) | D3 SMBJ15A TVS, C6 100 µF + C9 100 nF | INA219 VIN+ |
| +12_LOAD | +12 through R15 = 0.01 Ω (2512) | ≈ +12 − I·0.01 | measured by INA219 | Motors (pin 1), U3, U6 |
| +5 | TPS5430 U6, R12 = 10 k, R13 + R14 = 3 k + 220 = 3.22 k | **5.01 V** | Vout = 1.221 × (1 + 10k/3.22k) | Motor encoders (pin 6), buzzer, SW1 pos. 3 |
| +4 | TPS5430 U3, R7 = 10 k, R8 + R10 = 3.9 k + 510 = 4.41 k | **3.99 V** | Vout = 1.221 × (1 + 10k/4.41k) | J7 "BAT 1S" only |
| POWER | SW1 MTS-103: pos.3 = +5, pos.2 = POWER (common), pos.1 = USB VBUS | 5 V | ON-OFF-ON switch: battery / off / USB | AMS1117 |
| +3.3_MCU | AMS1117-3.3 (U4) | 3.3 V | D2 power LED (R11 220 Ω) | ESP32, CH340C, 74LVC14, INA219, IMU PCB, I²C pull-ups |

Consequences for firmware:
- The MCU can run from **USB with the battery absent** (SW1 on USB). INA219 bus voltage will then
  read ≈ 0 V → firmware must detect "no battery" and refuse to arm the motors.
- Everything on +12_LOAD (motors **and** both bucks) passes through the shunt → INA219 current =
  total board current from the battery (MCU current only when SW1 is on the battery side).

### 3S battery thresholds (to implement as config constants)

| State | Pack voltage | Per cell |
|---|---|---|
| Full | 12.6 V | 4.20 V |
| Nominal | 11.1 V | 3.70 V |
| Warning (beep) | 10.5 V | 3.50 V |
| Cut-off (disarm, brake) | 9.9 V | 3.30 V |
| Absolute minimum | 9.0 V | 3.00 V |

Use a filtered (e.g. 1 s moving average) voltage for thresholds because motor current causes sag.
Fuse limit ≈ 5 A total (2× 2.5 A PTC in parallel). Suggested software limits: warn 3.5 A, disarm if
> 4.5 A for > 200 ms (tunable).

---

## 4. ESP32 GPIO map (authoritative)

| GPIO | Module pin | Net | Direction | Function / notes |
|---|---|---|---|---|
| EN | 3 | EN | — | RESET button SW2, R2 10 k pull-up, C2 100 nF, auto-reset Q2 |
| GPIO0 | 25 | IO0 | in | BOOT button SW3 (active LOW, R4 10 k pull-up, C3 100 nF), auto-boot Q1. **Usable as user button after boot** |
| GPIO1 | 35 | ESP_TX | out | UART0 TX → CH340C RXD (USB console) |
| GPIO3 | 34 | ESP_RX | in | UART0 RX ← CH340C TXD |
| GPIO2 | 24 | STATUS | out | Status LED D1 via R18 220 Ω to GND — **active HIGH**. Strapping pin (keeps LOW at boot, OK) |
| GPIO4 | 26 | ROTATION1 | out | Motor 1 direction. R33 10 k pull-down |
| GPIO13 | 16 | ROTATION2 | out | Motor 2 direction. R34 10 k pull-down |
| GPIO14 | 13 | ROTATION3 | out | Motor 3 direction. R35 10 k pull-down |
| GPIO25 | 10 | PWM1 | out | Motor 1 speed PWM. R36 10 k pull-down |
| GPIO26 | 11 | PWM2 | out | Motor 2 speed PWM. R38 10 k pull-down |
| GPIO27 | 12 | PWM3 | out | Motor 3 speed PWM. R39 10 k pull-down |
| GPIO18 | 30 | BRAKE | out | **Shared** start/brake line for all 3 motors. R32 10 k pull-down (= braked at reset) |
| GPIO19 | 31 | BUZZER | out | Passive 9 mm buzzer via Q3 MMBT3904 (R16 1 k). Active HIGH, needs a tone (LEDC ~2–4 kHz). R37 10 k pull-down |
| GPIO21 | 33 | SDA | I/O | I²C SDA, R9 4.7 k pull-up to 3.3 V. Device: INA219 @ 0x40 |
| GPIO22 | 36 | SCL | out | I²C SCL, R17 4.7 k pull-up to 3.3 V |
| GPIO16 | 27 | SENS_RX | in | UART2 RX ← IMU PCB (J8 pin 2) |
| GPIO17 | 28 | SENS_TX | out | UART2 TX → IMU PCB (J8 pin 1) |
| GPIO34 | 6 | ENC_A1 | in | Motor 1 encoder A (via Schmitt 1Y). Input-only pin |
| GPIO35 | 7 | ENC_B1 | in | Motor 1 encoder B (via Schmitt 2Y). Input-only pin |
| GPIO36 (SENSOR_VP) | 4 | ENC_A2 | in | Motor 2 encoder A (via Schmitt 3Y). Input-only pin |
| GPIO39 (SENSOR_VN) | 5 | ENC_B2 | in | Motor 2 encoder B (via Schmitt 4Y). Input-only pin |
| GPIO32 | 8 | ENC_A3 | in | Motor 3 encoder A (via Schmitt 5Y) |
| GPIO33 | 9 | ENC_B3 | in | Motor 3 encoder B (via Schmitt 6Y) |
| GPIO5, 12, 15, 23 | 29, 14, 23, 37 | — | — | **Not connected** (free, no test pads). GPIO12/15 are strapping pins — leave unconfigured |

GPIO6–11 are used by the module flash — never touch. No ADC is used (battery is measured by INA219).

```c
// Reference pin header (firmware should generate board_pins.h from this)
#define PIN_STATUS_LED   2
#define PIN_BOOT_BTN     0
#define PIN_BRAKE       18   // shared, LOW = brake/stop all motors
#define PIN_BUZZER      19
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22
#define PIN_IMU_RX      16   // UART2
#define PIN_IMU_TX      17
//                 Motor1 Motor2 Motor3   (schematic numbering, see §6)
#define PIN_PWM    { 25,   26,    27 }
#define PIN_DIR    {  4,   13,    14 }
#define PIN_ENC_A  { 34,   36,    32 }
#define PIN_ENC_B  { 35,   39,    33 }
#define INA219_ADDR 0x40
```

---

## 5. Motor: Nidec 24H — wire colors and control semantics

### 5.1 Wire colour order (reverse-engineered)

The owner's original motor connector carries the 8 wires in this order, pin 1 → 8:
**red, black, yellow, white, green, blue, purple, grey**.

Mapping that connector against (a) the owner's previous board (8-pin 3.81 mm terminal "1727078":
1 = +12, 2 = GND, 3 = IO26 shared, 4 = IO32, 5 = IO4, 6 = +5, 7 = IO33, 8 = IO35 — same pin
functions as the RemRC Cubli: BRAKE = 26, PWM1 = 32, DIR1 = 4, ENC1 = 33/35) and (b) the vendor wiring
note for the Nidec 24H encoder version (red = +, black = −, yellow = start, white = PWM, green =
direction, remaining wires = encoder 5 V / A / B) gives a fully consistent result:

| Pin | Wire colour | Function | CubliV2 net | Notes |
|---|---|---|---|---|
| 1 | **Red** | Motor power + (12–24 V) | +12_LOAD | 100 µF + 100 nF per connector |
| 2 | **Black** | Power ground | GND | |
| 3 | **Yellow** | START / BRAKE | BRAKE (GPIO18, shared) | HIGH = run enabled, LOW = stop + electronic brake |
| 4 | **White** | PWM speed input | PWMx | ~20 kHz, **inverted**: LOW / 0 % duty = max speed, HIGH / 100 % = stop |
| 5 | **Green** | Direction (CW/CCW) | ROTATIONx | level selects direction (LOW = reverse per vendor note) |
| 6 | **Blue** | Encoder supply +5 V | +5 | 100 nF per connector |
| 7 | **Purple** | Encoder output | CHBx | → Schmitt → ENC_Bx |
| 8 | **Grey** | Encoder output | CHAx | → Schmitt → ENC_Ax |

The CubliV2 motor connectors (J2/J3/J4, Phoenix 1803484, 3.81 mm) use **exactly the same pin order**,
so the old connector plugs in directly. Pin 1 is the **square pad** on the PCB.

Confidence: pins 1–6 high (two independent sources agree). Purple vs grey = A vs B is only a
naming choice: swapping them just inverts the counting sign, so firmware must have a per-motor
`enc_invert` flag set by the bench test (§12), plus per-motor `dir_invert`.

### 5.2 Control semantics (verify on bench before balancing)

- **PWM:** LEDC, 20 kHz (motor starts reliably in ~16–26 kHz band), 10-bit resolution
  (80 MHz / 20 kHz = 4000 → max 11 bits). Command mapping: `duty = MAX − |u|` (inverted).
- **BRAKE (shared):** cannot brake a single motor — to stop one motor set its PWM to 100 % (stop).
  Releasing BRAKE releases all three.
- **Direction:** vendor recommends stopping before reversing. A reaction wheel reverses often;
  implement a small dead-band around zero command and allow direction change only when
  |commanded speed| is below a threshold (configurable), or measure whether the motor tolerates it.
- **Encoder:** 100 lines → **400 counts/rev** with ×4 quadrature decoding (verify by turning the
  wheel exactly 10 turns). Use the ESP32 **PCNT** peripheral (one unit per motor, both channels,
  glitch filter ≈ 1 µs) — no GPIO interrupts.
- Unknown: whether motor inputs have internal pull-ups to 5 V. Drive them push-pull 3.3 V
  (works on RemRC boards). Do not enable ESP32 internal pull-ups on these pins.

### 5.3 Safe sequencing (mandatory)

At reset all pull-downs give: BRAKE = LOW (braked — safe), PWM = LOW (**= full-speed command!**),
DIR = LOW. Therefore:
1. Configure LEDC on PWM1–3 with duty = 100 % (stop) **before** anything else touches BRAKE.
2. Keep BRAKE LOW while not armed, on fault, on low battery, on tilt-out, on watchdog.
3. Arm only after: battery OK, IMU healthy, cube near the target equilibrium, explicit user action.
4. Release BRAKE (HIGH) only after PWM is at stop level.

---

## 6. Motor numbering — schematic vs PCB silkscreen (IMPORTANT)

The silkscreen labels on the PCB do **not** match the schematic motor numbers. Verified from the
PcbDoc component records (designator, position) and the BOM:

| Schematic (nets) | Schematic designator | PCB silkscreen | Position on board (top view) | Pin 1 (square pad) |
|---|---|---|---|---|
| **Motor 1** — PWM1/GPIO25, ROTATION1/GPIO4, ENC GPIO34/35 | J2 | **MOTOR2** | top edge | left end |
| **Motor 2** — PWM2/GPIO26, ROTATION2/GPIO13, ENC GPIO36/39 | J3 | **MOTOR3** | left edge | top end |
| **Motor 3** — PWM3/GPIO27, ROTATION3/GPIO14, ENC GPIO32/33 | J4 | **MOTOR1** | right edge | bottom end |

Firmware rule: use the **schematic numbering (net names)** internally, and keep a configurable
mapping `motor index → physical cube axis (X/Y/Z)` in the config (it depends on how the board and
wheels are mounted in the cube — ask the owner).

---

## 7. Encoder signal chain

```
Motor CHx (pin 7/8) ─ 1 kΩ ─┬─ CHx_P ─► 74LVC14 input (Schmitt, inverting) ─► ENC_xx ─► ESP32
                            ├─ 10 kΩ pull-up to +3.3_MCU
                            └─ 1 nF to GND            (RC ≈ 1 µs, fc ≈ 160 kHz)
```

| Channel | Series R | Pull-up | Cap | 74LVC14 in → out | ESP32 |
|---|---|---|---|---|---|
| CHA1 | R20 | R26 | C12 | 1A → 1Y | ENC_A1 GPIO34 |
| CHB1 | R22 | R27 | C20 | 2A → 2Y | ENC_B1 GPIO35 |
| CHA2 | R24 | R28 | C31 | 3A → 3Y | ENC_A2 GPIO36 |
| CHB2 | R25 | R31 | C34 | 4A → 4Y | ENC_B2 GPIO39 |
| CHA3 | R23 | R29 | C32 | 5A → 5Y | ENC_A3 GPIO32 |
| CHB3 | R21 | R30 | C33 | 6A → 6Y | ENC_B3 GPIO33 |

- 74LVC14 (U5) powered from 3.3 V, 5 V-tolerant inputs; outputs push-pull → no ESP32 pull-ups
  needed (GPIO34–39 have none anyway).
- Both A and B are inverted → the quadrature direction is **unchanged** (only a 180° phase shift).
- Max expected rate: ~5000 rpm × 400 / 60 ≈ 33 k counts/s per motor — trivial for PCNT.

---

## 8. Other peripherals and connectors

| Ref (sch / silk) | Part | Pins | Firmware notes |
|---|---|---|---|
| J6 / XT-60 | XT60PW-M | P = battery +, N = GND | — |
| J8 / "IMU PCB" | 4-pin 2.54 mm header (schematic title "FPGA Connector") | 1 = SENS_TX (GPIO17), 2 = SENS_RX (GPIO16), 3 = GND, 4 = +3.3_MCU | External IMU board over **UART2**. Module type, baud rate and protocol are **unknown — ask the owner** |
| J7 / "BAT 1S" | Molex 1.25 mm 2-pin | 1 = +4 V, 2 = GND | Supplies an external board that expects a 1S cell. No signals |
| U7 | INA219AIDR | A0 = A1 = GND → **0x40**, VS = 3.3 V | VIN+ = +12, VIN− = +12_LOAD, shunt 0.01 Ω |
| LS1 / BUZZER | AT-0927-TT-2-R passive buzzer from +5 V (R19 51 Ω, flyback SS14) | GPIO19 | Use LEDC tones; different patterns for boot, arm, warning, fault |
| D1 | Red LED | GPIO2 | Status patterns (idle blink, armed solid, fault fast blink) |
| D2 | Red LED | +3.3 V | Power indicator, not controllable |
| SW2 / RESET | tactile | EN | — |
| SW3 / BOOT | tactile | GPIO0, active LOW | Single user button after boot: short press / long press / double press |
| SW1 / POWER | MTS-103 ON-OFF-ON | battery 5 V / off / USB 5 V → AMS1117 | — |

### INA219 recommended configuration

- Bus range 16 V (BRNG = 0) — 3S max 12.6 V fits.
- PGA ÷2 (±80 mV shunt) → ±8 A full scale with 0.01 Ω.
- Current_LSB = 0.25 mA → **CAL = trunc(0.04096 / (0.00025 × 0.01)) = 16384**; Power_LSB = 5 mW.
- 12-bit, 8–16 sample averaging, continuous shunt + bus; read at 10–50 Hz in a low-priority task.
- Bus voltage is measured at VIN− (= +12_LOAD) w.r.t. GND.

---

## 9. ESP32 resource plan (suggested)

| Resource | Use |
|---|---|
| LEDC timer 0, ch 0–2 | Motor PWM1–3, 20 kHz, 10-bit |
| LEDC timer 1, ch 3 | Buzzer tones |
| PCNT units 0–2 | Encoders 1–3 (quadrature ×4, glitch filter) |
| UART0 | USB console: CLI + telemetry |
| UART2 (GPIO16/17) | IMU PCB |
| I²C0 (GPIO21/22, 400 kHz) | INA219 |
| Core 1 | Control task (highest priority, fixed rate, e.g. 500 Hz–1 kHz) |
| Core 0 | Wi-Fi/BLE, telemetry, CLI, INA219, buzzer, LED |
| NVS | Calibration + tuning parameters |

---

## 10. Firmware requirements (what "upgraded vs RemRC" means)

1. Clean, modular C/C++ (drivers / estimation / control / app / comms), no logic in `main`.
2. Deterministic control loop on a dedicated core with timing statistics (jitter, overrun counter).
3. Hardware encoder decoding (PCNT) with wheel-speed estimation (filtered).
4. Attitude estimation with gyro-bias handling (complementary/Mahony or EKF) from the IMU PCB.
5. State-feedback (LQR-style) balance controller per axis: [tilt angle, tilt rate, wheel speed]
   (+ optional wheel-speed integral). Modes: **edge balance** (1 wheel) and **corner/vertex balance**
   (3 wheels, body-frame transform).
6. Supervisor state machine: BOOT → IDLE → CALIBRATE → READY → ARMED_EDGE / ARMED_VERTEX → FAULT.
7. Safety: brake on fault, tilt-out, low battery, over-current, IMU timeout, task watchdog.
8. Parameters and calibration stored in NVS; live tuning via USB CLI (and optional Wi-Fi/BLE).
9. Telemetry stream (CSV or binary) for plotting/tuning; optional OTA.
10. Bench-test commands that validate every pin in §4 and the wire mapping in §5.

---

## 11. Open questions (firmware must ask the owner — do not guess)

1. What is on the **IMU PCB** (sensor model, e.g. MPU6050 / ICM-42688 / BNO08x / custom FPGA
   board), UART baud rate, frame format, output rate, and is it orientation data or raw IMU?
2. What does the **J7 "BAT 1S" (+4 V)** output power exactly?
3. Framework preference: ESP-IDF (recommended) or Arduino-ESP32, both via PlatformIO?
4. Mechanical data: cube edge length, total mass, wheel inertia (or wheel mass/diameter), wheel
   and board orientation inside the cube (which motor spins about which axis), IMU mounting axes.
5. Exact Nidec 24H model number (e.g. 24H404H…), rated voltage and max rpm.
6. Wireless tuning wanted (Wi-Fi web dashboard / BLE) or USB only?

## 12. Bench bring-up checklist (before any balancing)

1. No motors: measure +5 V ≈ 5.0 V, +4 V ≈ 4.0 V, +3.3 V; INA219 reads pack voltage.
2. Motor connector continuity: pin 1 ↔ +12_LOAD, pin 2 ↔ GND, pin 6 ↔ +5.
3. One motor, wheel free, low speed: `brake off` → motor spins only when PWM < 100 %; confirms
   yellow = brake, white = PWM (inverted).
4. Toggle DIR → rotation reverses (confirms green).
5. Turn wheel by hand 10 turns → ≈ 4000 counts (confirms blue/purple/grey + 400 CPR); set
   `enc_invert` so positive command ⇒ positive count.
6. Repeat for the other two motors; record mapping schematic Motor n ↔ silkscreen MOTORm ↔ cube axis.
