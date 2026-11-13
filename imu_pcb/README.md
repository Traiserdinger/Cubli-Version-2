# IMU PCB patch — frame v2 for CubliV2

The CubliV2 balance controller needs the body **angular rate**; the stock IMU PCB
firmware (`DAKTNC_Nhom2_MX`, STM32F103) only sends Euler angles (frame v1). This patch adds
a second frame type without changing the default behaviour: after power-up the board
still sends frame v1 at 57600 baud, so other users of the board (FPGA project) are
unaffected.

## What changes

| File | Change |
|---|---|
| `Core/Inc/app_config.h` | `CFG_UART_CUBLI_BAUD` (460800), `CFG_CUBLI_DISABLE_MAG` (1) |
| `Core/Inc/comm_protocol.h` | frame v2 definition (section 6 of the header comment), new API |
| `Core/Src/comm_protocol.c` | CRC-16, v2 packing, commands `CUBLI` / `FRAME1` |
| `Core/Src/app_main.c` | fills frame v2 every sample, `REINIT` command, magnetometer not fused in v2 mode |

`patched/` holds the complete modified files, `original/` the files they replace, and
`cubliv2_frame_v2.patch` the unified diff (`diff -u original patched`).

## Frame v2 (28 bytes, little-endian)

| Bytes | Field | Type | Meaning |
|---|---|---|---|
| 0–1 | header | `AA 56` | |
| 2 | SEQ | u8 | +1 per frame |
| 3 | STATUS | u8 | bit0 EKF ready, bit1 IMU1 ok, bit2 IMU2 ok, bit3 accel update rejected, bit4 magnetometer fused, bit5 sampling on the INT pin |
| 4–11 | q0..q3 | i16 ×4 | quaternion body → world (NWU), scale 1/32767 |
| 12–17 | gx, gy, gz | i16 ×3 | bias-corrected gyro, 1 mrad/s |
| 18–23 | ax, ay, az | i16 ×3 | accelerometer, 1/4096 g |
| 24–25 | T_US | u16 | sample time, µs (wraps) |
| 26–27 | CRC | u16 | CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over bytes 0–25 |

At 460800 baud one frame takes 0.61 ms, so all 1000 EKF results per second are sent.

## Commands (ASCII, CR/LF, case-insensitive)

| Command | Effect |
|---|---|
| `CUBLI` | after the current frame and 20 ms of silence: frame v2 at 460800 baud |
| `FRAME1` | back to frame v1 at 57600 baud |
| `REINIT` | restart the EKF: re-learns attitude and gyro bias (keep the cube still ~1 s) |

The ESP32 sends `CUBLI` automatically when it sees frame v1, and `REINIT` on `calib gyro`.

## How to apply

1. Copy the four files from `patched/` over the same files in `DAKTNC_Nhom2_MX/Core/`
   (or apply `cubliv2_frame_v2.patch` with `patch -p1` from inside `Core/` after
   renaming the paths).
2. Build and flash with STM32CubeIDE as usual.
3. Check on the ESP32 console: `imu` must show `link v2 @460800` and `ekf 1`.

Verified here (no STM32 hardware available): both C files compile with
`gcc -Wall -Wextra -Werror`, and a host test drives the patched `comm_protocol.c`
(command `cubli` → drain → baud change → frame v2) and decodes the bytes with the ESP32
parser (`firmware/lib/estimation/imu_frame.cpp`). Timing on the STM32 (the extra
packing costs a few tens of µs per sample) must be checked with `g_app_diag.step_us_max`.

The new comments in the patched files are in Vietnamese to match the original project.
