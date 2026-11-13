// Parser for the IMU PCB UART frames (context.md §8 "IMU PCB").
//
// Frame v1 (stock IMU PCB firmware, 16 bytes, little-endian):
//   0 0xAA | 1 0x55 | 2 SEQ | 3-4 ROLL | 5-6 PITCH | 7-8 YAW (int16, 0.01 deg)
//   9-10 mV (u16) | 11-12 mA (i16) | 13-14 mW (u16) | 15 CRC-8/0x07 over 0..14
//
// Frame v2 (CubliV2 extension, imu_pcb/ patch, 28 bytes, little-endian):
//   0 0xAA | 1 0x56 | 2 SEQ | 3 STATUS
//   4-11  quaternion w,x,y,z (int16, 1/32767)        body -> world (NWU)
//   12-17 gyro x,y,z (int16, 1 mrad/s)              bias-corrected, body frame
//   18-23 accel x,y,z (int16, 1/4096 g)             body frame
//   24-25 sample time (u16, microseconds, wraps)
//   26-27 CRC-16/CCITT-FALSE over bytes 0..25
#pragma once

#include <cstddef>
#include <cstdint>

#include "math3.h"

namespace cubli {

inline constexpr uint8_t kImuHeader0 = 0xAA;
inline constexpr uint8_t kImuHeaderV1 = 0x55;
inline constexpr uint8_t kImuHeaderV2 = 0x56;
inline constexpr size_t kImuFrameV1Size = 16;
inline constexpr size_t kImuFrameV2Size = 28;

namespace imu_status {
inline constexpr uint8_t kEkfReady = 0x01;
inline constexpr uint8_t kImu1Ok = 0x02;
inline constexpr uint8_t kImu2Ok = 0x04;
inline constexpr uint8_t kAccelRejected = 0x08;
inline constexpr uint8_t kMagFused = 0x10;
inline constexpr uint8_t kDrdyClock = 0x20;
}  // namespace imu_status

struct ImuFrameV1 {
    uint8_t seq = 0;
    float roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
    float supply_v = 0, supply_a = 0, supply_w = 0;
};

struct ImuFrameV2 {
    uint8_t seq = 0;
    uint8_t status = 0;
    Quat q;
    Vec3 gyro;   // rad/s
    Vec3 accel;  // g
    uint16_t t_us = 0;
};

/// Encode a v2 frame (used by unit tests and by the reference IMU PCB patch).
void encode_frame_v2(const ImuFrameV2& f, uint8_t out[kImuFrameV2Size]);

/// Byte-wise frame parser. Feed every received byte; at most one frame completes per byte.
class ImuFrameParser {
public:
    enum class Result { kNone, kFrameV1, kFrameV2 };

    Result feed(uint8_t byte);

    const ImuFrameV1& v1() const { return v1_; }
    const ImuFrameV2& v2() const { return v2_; }

    uint32_t crc_errors() const { return crc_errors_; }
    uint32_t frames_v1() const { return frames_v1_; }
    uint32_t frames_v2() const { return frames_v2_; }
    /// Frames missing according to the sequence counter (per frame type).
    uint32_t seq_gaps() const { return seq_gaps_; }
    void reset_stats() { crc_errors_ = frames_v1_ = frames_v2_ = seq_gaps_ = 0; have_seq_ = false; }

private:
    void decode_v1();
    void decode_v2();
    void track_seq(uint8_t seq);

    enum class State { kHunt, kHeader, kBody } state_ = State::kHunt;
    uint8_t buf_[kImuFrameV2Size] = {};
    size_t len_ = 0, need_ = 0;
    ImuFrameV1 v1_;
    ImuFrameV2 v2_;
    uint32_t crc_errors_ = 0, frames_v1_ = 0, frames_v2_ = 0, seq_gaps_ = 0;
    uint8_t last_seq_ = 0;
    bool have_seq_ = false;
};

}  // namespace cubli
