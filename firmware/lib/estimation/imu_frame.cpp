#include "imu_frame.h"

#include <cmath>

#include "crc.h"

namespace cubli {
namespace {

int16_t get_i16(const uint8_t* p) { return static_cast<int16_t>(p[0] | (p[1] << 8)); }
uint16_t get_u16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
void put_u16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>(v >> 8);
}
int16_t sat_i16(float v) {
    const float r = std::nearbyint(v);
    if (r > 32767.0f) return 32767;
    if (r < -32768.0f) return -32768;
    return static_cast<int16_t>(r);
}

constexpr float kQuatScale = 32767.0f;
constexpr float kGyroScale = 1000.0f;   // LSB = 1 mrad/s
constexpr float kAccelScale = 4096.0f;  // LSB = 1/4096 g

}  // namespace

void encode_frame_v2(const ImuFrameV2& f, uint8_t out[kImuFrameV2Size]) {
    out[0] = kImuHeader0;
    out[1] = kImuHeaderV2;
    out[2] = f.seq;
    out[3] = f.status;
    const float q[4] = {f.q.w, f.q.x, f.q.y, f.q.z};
    for (int i = 0; i < 4; ++i) put_u16(&out[4 + 2 * i], static_cast<uint16_t>(sat_i16(q[i] * kQuatScale)));
    for (int i = 0; i < 3; ++i) put_u16(&out[12 + 2 * i], static_cast<uint16_t>(sat_i16(f.gyro[i] * kGyroScale)));
    for (int i = 0; i < 3; ++i) put_u16(&out[18 + 2 * i], static_cast<uint16_t>(sat_i16(f.accel[i] * kAccelScale)));
    put_u16(&out[24], f.t_us);
    put_u16(&out[26], crc16_ccitt(out, 26));
}

ImuFrameParser::Result ImuFrameParser::feed(uint8_t byte) {
    switch (state_) {
        case State::kHunt:
            if (byte == kImuHeader0) {
                buf_[0] = byte;
                state_ = State::kHeader;
            }
            return Result::kNone;

        case State::kHeader:
            if (byte == kImuHeaderV1 || byte == kImuHeaderV2) {
                buf_[1] = byte;
                len_ = 2;
                need_ = (byte == kImuHeaderV1) ? kImuFrameV1Size : kImuFrameV2Size;
                state_ = State::kBody;
            } else if (byte != kImuHeader0) {
                state_ = State::kHunt;
            }
            return Result::kNone;

        case State::kBody:
            buf_[len_++] = byte;
            if (len_ < need_) return Result::kNone;
            state_ = State::kHunt;
            if (need_ == kImuFrameV1Size) {
                if (crc8_07(buf_, kImuFrameV1Size - 1) != buf_[kImuFrameV1Size - 1]) {
                    ++crc_errors_;
                    return Result::kNone;
                }
                decode_v1();
                ++frames_v1_;
                return Result::kFrameV1;
            }
            if (crc16_ccitt(buf_, 26) != get_u16(&buf_[26])) {
                ++crc_errors_;
                return Result::kNone;
            }
            decode_v2();
            ++frames_v2_;
            return Result::kFrameV2;
    }
    return Result::kNone;
}

void ImuFrameParser::track_seq(uint8_t seq) {
    if (have_seq_) {
        const uint8_t expected = static_cast<uint8_t>(last_seq_ + 1);
        seq_gaps_ += static_cast<uint8_t>(seq - expected);
    }
    last_seq_ = seq;
    have_seq_ = true;
}

void ImuFrameParser::decode_v1() {
    v1_.seq = buf_[2];
    v1_.roll_deg = get_i16(&buf_[3]) * 0.01f;
    v1_.pitch_deg = get_i16(&buf_[5]) * 0.01f;
    v1_.yaw_deg = get_i16(&buf_[7]) * 0.01f;
    v1_.supply_v = get_u16(&buf_[9]) * 0.001f;
    v1_.supply_a = get_i16(&buf_[11]) * 0.001f;
    v1_.supply_w = get_u16(&buf_[13]) * 0.001f;
    track_seq(v1_.seq);
}

void ImuFrameParser::decode_v2() {
    v2_.seq = buf_[2];
    v2_.status = buf_[3];
    Quat q{get_i16(&buf_[4]) / kQuatScale, get_i16(&buf_[6]) / kQuatScale,
           get_i16(&buf_[8]) / kQuatScale, get_i16(&buf_[10]) / kQuatScale};
    const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n > 0.5f) {
        q.w /= n; q.x /= n; q.y /= n; q.z /= n;
    } else {
        q = Quat{};
        v2_.status = static_cast<uint8_t>(v2_.status & ~imu_status::kEkfReady);
    }
    v2_.q = q;
    for (int i = 0; i < 3; ++i) v2_.gyro[i] = get_i16(&buf_[12 + 2 * i]) / kGyroScale;
    for (int i = 0; i < 3; ++i) v2_.accel[i] = get_i16(&buf_[18 + 2 * i]) / kAccelScale;
    v2_.t_us = get_u16(&buf_[24]);
    track_seq(v2_.seq);
}

}  // namespace cubli
