# CubliV2 firmware

ESP32-WROOM-32E firmware for the CubliV2 reaction-wheel cube: edge and vertex balancing with
LQR state feedback, an identified plant model, automatic gain design, USB and Bluetooth (BLE)
command line, and layered safety. Hardware facts come from [`../context.md`](../context.md).

> **Status:** builds without warnings, 28 unit/simulation tests pass on the PC. **Nothing has run
> on real hardware yet.** Follow the bring-up order below; every step that moves a motor must be
> done with the wheel free and the cube held or secured.

## Build and flash

```bash
cd firmware
pio run                     # build (PlatformIO, espressif32 6.13.0 = ESP-IDF 5.5.3)
pio run -t upload           # flash over USB-C
pio device monitor          # console, 921600 baud (DTR/RTS released: no reset)
pio test -e native          # PC tests incl. closed-loop simulation
```

Any serial terminal works at 921600 8N1 if DTR and RTS are **off** (they drive EN/IO0 through
the auto-reset circuit). Bluetooth: device **CubliV2**, Nordic UART Service — use any "BLE UART"
app (nRF Toolbox, Serial Bluetooth Terminal, LightBlue) with **CR or LF line endings enabled**.
USB and BLE accept the same commands.

## Layout

```
include/board_pins.h   pin map from context.md §4 (static checks: no flash/strapping pins)
include/hw_config.h    fixed hardware constants (PWM, CPR, battery limits)
lib/core/              math, CRC, lock-free ring / seqlock, filters          (no ESP-IDF)
lib/estimation/        IMU frame parser, wheel speed, INA219 arithmetic      (no ESP-IDF)
lib/control/           plant model, LQR, balance controller, system id       (no ESP-IDF)
lib/logic/             parameters, calibration, supervisor, CLI tokenizer    (no ESP-IDF)
src/hal/               motor PWM/DIR/BRAKE, PCNT encoders, INA219, IMU UART, buzzer, NVS
src/comms/             USB/BLE output sinks, BLE Nordic UART Service
src/app/               control, power, UI, console and telemetry tasks
src/main.cpp           safe init order, task start
test/                  native tests; test_control/sim_plant.h = physical cube simulation
tools/                 lqr_design.py, plot_telemetry.py
../imu_pcb/            patch for the IMU PCB firmware (frame v2)
```

## Tasks

| Task | Core | Priority | Rate | Work |
|---|---|---|---|---|
| control | 1 | 23 | 1 kHz, paced by IMU frames (≥ 500 Hz without) | IMU link, encoders, supervisor, controller, outputs, recording |
| esp_timer (deadman) | 0 | 22 | 500 Hz | brakes at GPIO level if the control loop stalls > 20 ms with the brake released |
| power | 0 | 5 | 50 Hz | INA219, 1 s averages, battery and current limits |
| console | 0 | 4 | events | USB/BLE commands, event messages, NVS saving, identification + LQR |
| ui | 0 | 3 | 100 Hz | LED, buzzer, BOOT button |
| telemetry | 0 | 2 | stream | `$T` lines to USB / BLE |

The control task never prints, allocates or blocks on another task: it reads parameters through
a seqlock, receives commands from a queue, and publishes status, events and telemetry through
lock-free buffers. The task watchdog (1 s) resets the chip if it hangs; the hardware pull-down
then brakes all motors.

## IMU PCB link

The ESP32 searches 460800/57600/921600/230400/115200 baud for valid frames. On frame v1 (stock
IMU PCB firmware) it sends `CUBLI` to switch the IMU PCB to frame v2 at 460800 baud: quaternion,
bias-corrected gyro, accelerometer at 1 kHz with CRC-16. Frame v1 has no angular rate, so arming
needs the patch in [`../imu_pcb/`](../imu_pcb/README.md). `imu` shows the link state.

## State machine

```
BOOT -> IDLE <-> READY -> ARMING -> ARMED_EDGE / ARMED_VERTEX
         |  ^      |        ^  |            |
         v  |      v        |  +- timeout   +- tilt > tilt_lim (fell) -> READY (or ARMING if rearm=1)
      CALIBRATE  BENCH      +---------------- rearm ---------------/
any state -- latched fault --> FAULT -- "fault clear" once the cause is gone --> IDLE
```

READY = IMU frame v2 with EKF ready, battery present and above cut-off, motor axes calibrated,
at least one balance point stored. In ARMING the motors stay braked until the cube is held within
`arm_cap` (2°) of a stored point and still for `arm_hold` (0.3 s); then the matching controller
starts by itself.

## Safety

| Layer | Behaviour |
|---|---|
| Reset | BRAKE pulled LOW (braked). `MotorOutput::init()` runs first: BRAKE LOW, PWM pins HIGH (= stop) as GPIO, then LEDC at 100 % duty |
| Brake release | only while every PWM output is at the stop level; commands start the next cycle |
| Faults (latched) | over-current (4.5 A for 200 ms), wheel overspeed, deadman, repeated overruns, NaN; while motors run also battery below cut-off (1 s average), battery removed, IMU frame older than `imu_tout` (10 ms) |
| Stop strategy | PWM to stop at once; BRAKE stays released (coast) while a wheel is faster than `brake_rads` for at most `coast_s` (1.5 s), then BRAKE LOW. Avoids jerking the cube with spinning wheels. Deadman: brake immediately |
| Direction | DIR reversal only below `dir_sw_rads` (default: always allowed, as RemRC does) |
| Arming | never automatic at boot; needs `arm` or holding BOOT 1 s, then a still cube at a calibrated point |
| Flash writes | `param save` and calibration saving are refused / deferred while the brake is released (flash writes stall both cores) |

## Bring-up order

1. **USB only, no motors.** `status`, `bat` (INA219 reachable, "absent (USB only)"), `imu`.
2. **Battery, no motors.** `bat` shows the pack voltage. Optional: `calib bat <multimeter volts>`,
   `calib current` with motors stopped. `param save`.
3. **Reset-level check (once, wheel removed or clamped).** Hold RESET, connect one motor,
   measure pins 3, 4, 5 of its connector: pin 3 (BRAKE) must be LOW. While RESET is held PWM is
   LOW (= full-speed command), so if BRAKE reads above ~0.8 V the motor may start: release the
   battery, and report it (R32 must be made smaller).
4. **Wiring per motor:** `wiretest <n>`. Step A by hand (`enc zero`, 10 turns, `enc` → ±4000),
   step B automatic (PWM, DIR, BRAKE, encoder sign). Apply the suggested `m<n>_enc_inv`,
   `param save`. Repeat for motors 1–3. Mapping: Motor 1 = J2 = MOTOR2, Motor 2 = J3 = MOTOR3,
   Motor 3 = J4 = MOTOR1.
5. **IMU PCB patch** flashed: `imu` shows `link v2 @460800`, `ekf 1`. `calib gyro` with the cube
   still re-learns the gyro bias.
6. **Motor axes:** hold the cube loosely in the air, `calib axes` (three short kicks).
7. **Balance points:** hold the cube exactly balanced on an edge, `calib point` (or double-click
   BOOT). Repeat for each edge/vertex you want (up to 8). `calib show`.
8. **Back-EMF per motor:** cube resting on a face, `sysid motor <n>`, `autotune apply` (sets
   `m<n>_kv`). Repeat for all three.
9. **First balance on an edge:** `arm` (or hold BOOT 1 s), hold the cube at the edge point; it
   takes over. `tlm 200` + `tools/plot_telemetry.py` to watch.
10. **Autotune:** while it balances on the edge, `autotune run` → identified model (a, b, c) and
    LQR gains; `autotune apply`; `param save`. Same on the vertex. If the default gains cannot
    hold the cube at all, use `autotune drop` (open-loop identification while it falls on an edge).
11. **Manual tuning** at any time: `param set e_k1 …` (active immediately), or change the LQR
    weights `lqr_th`, `lqr_rate`, `lqr_s`, `lqr_p`, `lqr_u` and run `lqr edge apply`.

## Commands

| Command | Purpose |
|---|---|
| `help`, `version`, `status`, `bat`, `enc [zero]`, `imu [send <cmd>\|restart]` | information |
| `bench on\|off`, `brake on\|off`, `motor <1-3> <-1000..1000>`, `stop`, `beep`, `wiretest <1-3>` | bench (motor commands stop after 3 s) |
| `param list [filter]\|get\|set\|save\|load\|reset` | parameters (NVS) |
| `calib show\|axes\|point\|delete <i>\|clear [all]\|gyro\|bat <V>\|current\|axis <n> <x> <y> <z>` | calibration (saved automatically) |
| `arm`, `disarm`, `fault [clear]`, `mode auto\|edge\|vertex` | operation |
| `sysid motor <n>`, `autotune run\|drop\|apply\|cancel`, `lqr [edge\|vertex] [apply]` | tuning |
| `log [ms]`, `tlm [ble] <hz>\|off`, `reboot` | data |

BOOT button: hold 1 s = arm / disarm, one press = disarm, double press = store balance point.
LED: idle short blink, ready double blink, arming fast, armed solid, bench 1 Hz, fault very fast.

## Control design

All vectors are in the IMU PCB body frame. `up` = world vertical from the IMU quaternion;
`up_eq` = stored balance point; tilt = `up × up_eq` (small rotation away from equilibrium).
`axis[k]` = body-acceleration direction for a positive command on motor k (from `calib axes`).

Per balancing axis (edge: the motor axis; vertex: two horizontal axes) the model is

```
x = [theta, theta_dot, s, p]          s = wheel speed, p = wheel angle since arming
theta_ddot = a*theta + b*tau          a = M g l / J,  b = Km / J,  c = Km / Iw
s_dot      = a*theta + (b+c)*tau      tau = u - kv*s  (back-EMF removed by the output stage)
```

`tau = -K x` with K from the discrete infinite-horizon LQR (1 kHz, Bryson weights, doubling
algorithm). At the vertex the two axis commands and a yaw damping term form a body-torque vector
that is mapped to the three motors with the inverse of the measured axis matrix.

Why this beats hand-tuned PID gains: the gains are optimal for the *identified* model, the
back-EMF feed-forward makes the actuator linear, the wheel-angle state removes steady wheel drift,
and the balance point trims itself (`trim_tau`) toward the true centre of mass.

Default gains are the LQR solution of a nominal 15 cm / 1.2 kg cube (`e_a`… `v_c`); they balance
the simulated cube but must be replaced by `autotune` on the real one.

Simulation results (`pio test -e native -v`): edge and vertex recover from 3° to < 0.03°;
stable with ±30–40 % model error and 30 % back-EMF error; identification recovers a and b within
~7 %, c within ~20 %; the self-trim removes a 1.5° calibration error.

## Telemetry

`tlm 200` streams `$T,t_ms,state,brake,th0_deg,th1_deg,rate0,rate1,w1,w2,w3,u1,u2,u3,V,A,exec_us`
(USB up to 500 Hz, BLE up to 50 Hz). `log 2000` records 500 Hz controller data and dumps `$L`
lines. `tools/plot_telemetry.py` captures and plots both.

## Not verified / open (see context.md §11)

- Motor input behaviour (PWM linearity, coasting vs. holding at PWM stop with BRAKE released,
  internal pull-ups) — characterise with `wiretest`, `sysid motor`.
- Encoder 400 CPR, IMU mounting orientation, real loop timing (`status` shows loop rate, max
  execution time and overruns), BLE throughput.
- The IMU PCB patch compiles and is cross-checked against the ESP32 parser, but has not run on
  the STM32.
