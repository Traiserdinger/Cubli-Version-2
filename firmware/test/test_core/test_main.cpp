// Unit tests for hardware-independent modules.
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include <unity.h>

#include <cmath>
#include <cstring>
#include <initializer_list>
#include <thread>

#include "calibration.h"
#include "cli_parse.h"
#include "crc.h"
#include "filters.h"
#include "imu_frame.h"
#include "ina219_math.h"
#include "params.h"
#include "supervisor.h"
#include "sync.h"
#include "wheel_speed.h"

using namespace cubli;

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- crc
void test_crc_check_values() {
    const uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16_ccitt(s, sizeof s));
    TEST_ASSERT_EQUAL_HEX8(0xF4, crc8_07(s, sizeof s));  // CRC-8/SMBUS check value
}

// ---------------------------------------------------------------- ring / seqlock
void test_spsc_ring_order_and_full() {
    SpscRing<int, 8> r;
    for (int i = 0; i < 7; ++i) TEST_ASSERT_TRUE(r.push(i));
    TEST_ASSERT_FALSE(r.push(99));
    TEST_ASSERT_EQUAL_UINT32(1, r.dropped());
    int v;
    for (int i = 0; i < 7; ++i) {
        TEST_ASSERT_TRUE(r.pop(v));
        TEST_ASSERT_EQUAL(i, v);
    }
    TEST_ASSERT_FALSE(r.pop(v));
}

void test_spsc_ring_threads() {
    SpscRing<uint32_t, 64> r;
    constexpr uint32_t kN = 200000;
    std::thread prod([&] {
        for (uint32_t i = 0; i < kN;) {
            if (r.push(i)) ++i;
        }
    });
    uint32_t expect = 0, v = 0;
    while (expect < kN) {
        if (r.pop(v)) {
            TEST_ASSERT_EQUAL_UINT32(expect, v);
            ++expect;
        }
    }
    prod.join();
}

struct Blob {
    uint32_t a, b, c, d;
};
void test_seqlock_consistent() {
    SeqLock<Blob> sl;
    std::thread w([&] {
        for (uint32_t i = 1; i < 200000; ++i) sl.write(Blob{i, i, i, i});
    });
    for (int n = 0; n < 200000; ++n) {
        Blob b{};
        if (sl.read(b)) {
            TEST_ASSERT_EQUAL_UINT32(b.a, b.b);
            TEST_ASSERT_EQUAL_UINT32(b.a, b.d);
        }
    }
    w.join();
}

// ---------------------------------------------------------------- filters
void test_lowpass_step() {
    LowPass1 lp;
    lp.update(0.0f, 10.0f, 0.001f);
    float y = 0;
    for (int i = 0; i < 1000; ++i) y = lp.update(1.0f, 10.0f, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, y);
    MovingAverage<4> ma;
    for (float x : {4.0f, 8.0f, 0.0f, 0.0f, 4.0f}) ma.update(x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.0f, ma.value());
}

// ---------------------------------------------------------------- IMU frames
static void make_v1(uint8_t* f, uint8_t seq, int16_t roll) {
    std::memset(f, 0, 16);
    f[0] = 0xAA; f[1] = 0x55; f[2] = seq;
    f[3] = static_cast<uint8_t>(roll & 0xFF); f[4] = static_cast<uint8_t>((roll >> 8) & 0xFF);
    f[9] = 0xE8; f[10] = 0x0F;  // 4072 mV
    f[15] = crc8_07(f, 15);
}

void test_parser_v1_and_v2_with_garbage() {
    ImuFrameParser p;
    uint8_t buf[200];
    size_t n = 0;
    const uint8_t junk[] = {0x00, 0xAA, 0x13, 0xAA, 0xAA};
    std::memcpy(buf + n, junk, sizeof junk); n += sizeof junk;
    make_v1(buf + n, 7, -1234); n += 16;
    ImuFrameV2 f;
    f.seq = 8;
    f.status = imu_status::kEkfReady | imu_status::kImu1Ok;
    const float h = std::sqrt(0.5f);
    f.q = Quat{h, h, 0, 0};
    f.gyro = {0.5f, -1.25f, 3.0f};
    f.accel = {0.0f, 0.7071f, 0.7071f};
    f.t_us = 54321;
    encode_frame_v2(f, buf + n); n += kImuFrameV2Size;
    uint8_t bad[kImuFrameV2Size];
    encode_frame_v2(f, bad);
    bad[10] ^= 0x01;  // corrupt
    std::memcpy(buf + n, bad, sizeof bad); n += sizeof bad;
    f.seq = 10;  // one frame (9) missing
    encode_frame_v2(f, buf + n); n += kImuFrameV2Size;

    int v1 = 0, v2 = 0;
    for (size_t i = 0; i < n; ++i) {
        const auto r = p.feed(buf[i]);
        if (r == ImuFrameParser::Result::kFrameV1) {
            ++v1;
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, -12.34f, p.v1().roll_deg);
            TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.072f, p.v1().supply_v);
        }
        if (r == ImuFrameParser::Result::kFrameV2) {
            ++v2;
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, h, p.v2().q.w);
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, -1.25f, p.v2().gyro.y);
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.7071f, p.v2().accel.z);
            TEST_ASSERT_EQUAL_UINT16(54321, p.v2().t_us);
        }
    }
    TEST_ASSERT_EQUAL(1, v1);
    TEST_ASSERT_EQUAL(2, v2);
    TEST_ASSERT_EQUAL_UINT32(1, p.crc_errors());
    TEST_ASSERT_EQUAL_UINT32(1, p.seq_gaps());  // 7, 8, (9 corrupt), 10
}

void test_up_vector_from_quaternion() {
    // 90 deg about world X: body +Y now points up.
    const float h = std::sqrt(0.5f);
    const Vec3 up = up_in_body(Quat{h, h, 0, 0});
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, up.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, up.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, up.z);
}

// ---------------------------------------------------------------- INA219
void test_ina219_math() {
    // 0.05 ohm, 6 A -> 300 mV -> PGA /8 (320 mV)
    TEST_ASSERT_EQUAL_UINT8(3, ina219::select_pga(6.0f, 0.05f));
    TEST_ASSERT_EQUAL_UINT8(1, ina219::select_pga(6.0f, 0.01f));  // 60 mV -> 80 mV
    TEST_ASSERT_EQUAL_HEX16(0x1DDF, ina219::make_config(3));
    // 12.000 V on the bus: 3000 * 4 mV, CNVR set
    const auto b = ina219::decode_bus(static_cast<uint16_t>((3000u << 3) | 0x2u));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 12.0f, b.volts);
    TEST_ASSERT_TRUE(b.conversion_ready);
    TEST_ASSERT_FALSE(b.overflow);
    // 100 mV across 0.05 ohm = 2 A; -2 A as two's complement
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f, ina219::decode_current(10000, 0.05f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -2.0f, ina219::decode_current(static_cast<uint16_t>(-10000), 0.05f));
}

// ---------------------------------------------------------------- wheel speed
void test_wheel_speed_constant_rate() {
    WheelSpeedEstimator e;
    e.reset(0, 0u);
    const double w = 100.0;  // rad/s
    float s = 0;
    for (int i = 1; i <= 500; ++i) {
        const double t = i * 0.001;
        const int32_t c = static_cast<int32_t>(std::floor(w * t * 400.0 / (2 * M_PI)));
        s = e.update(c, static_cast<uint32_t>(i) * 1000u, 8, 80.0f, 400);
    }
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 100.0f, s);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 50.0f, e.angle(400));
}

// ---------------------------------------------------------------- params
void test_params_defaults_in_range_and_roundtrip() {
    Params p;
    params_set_defaults(p);
    for (size_t i = 0; i < param_count(); ++i) {
        const ParamInfo& pi = param_info(i);
        TEST_ASSERT_TRUE_MESSAGE(pi.def >= pi.min && pi.def <= pi.max, pi.name);
    }
    TEST_ASSERT_NULL(param_set_text(p, "shunt_mohm", "47.5"));
    TEST_ASSERT_NULL(param_set_text(p, "m2_enc_inv", "on"));
    TEST_ASSERT_NOT_NULL(param_set_text(p, "shunt_mohm", "0"));
    TEST_ASSERT_NOT_NULL(param_set_text(p, "nope", "1"));
    TEST_ASSERT_NOT_NULL(param_set_text(p, "wheel_win", "abc"));
    TEST_ASSERT_NULL(param_set_text(p, "wheel_win", "12.4"));
    TEST_ASSERT_EQUAL_FLOAT(12.0f, p.wheel_win);

    static uint8_t blob[4096];
    const size_t n = params_serialize(p, blob, sizeof blob);
    TEST_ASSERT_EQUAL(params_blob_size(), n);
    Params q;
    TEST_ASSERT_EQUAL(static_cast<int>(param_count()), params_deserialize(q, blob, n));
    TEST_ASSERT_EQUAL_FLOAT(47.5f, q.shunt_mohm);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, q.m2_enc_inv);
    blob[20] ^= 0x40;
    TEST_ASSERT_EQUAL(-1, params_deserialize(q, blob, n));
    TEST_ASSERT_EQUAL_FLOAT(50.0f, q.shunt_mohm);  // back to defaults
}

void test_calibration_roundtrip_and_points() {
    CalibrationData c;
    c.axes_valid = 1;
    const Vec3 edge_up = normalized(Vec3{1, 1, 0});
    TEST_ASSERT_EQUAL(0, store_point(c, edge_up, PointKind::kEdge, 2, 0.17f));
    TEST_ASSERT_EQUAL(0, store_point(c, normalized(Vec3{1, 1.05f, 0}), PointKind::kEdge, 2, 0.17f));  // merged
    TEST_ASSERT_EQUAL(1, store_point(c, normalized(Vec3{1, 1, 1}), PointKind::kVertex, -1, 0.17f));
    TEST_ASSERT_EQUAL(2, c.point_count());
    float ang;
    TEST_ASSERT_EQUAL(1, find_nearest_point(c, normalized(Vec3{1, 1, 0.95f}), 0, &ang));
    TEST_ASSERT_EQUAL(0, find_nearest_point(c, normalized(Vec3{1, 1, 0.95f}), 1, &ang));
    uint8_t blob[512];
    const size_t n = calib_serialize(c, blob, sizeof blob);
    CalibrationData d;
    TEST_ASSERT_TRUE(calib_deserialize(d, blob, n));
    TEST_ASSERT_EQUAL(2, d.point_count());
    blob[n - 1] ^= 1;
    TEST_ASSERT_FALSE(calib_deserialize(d, blob, n));
}

// ---------------------------------------------------------------- supervisor
static SupervisorInputs ready_inputs() {
    SupervisorInputs in;
    in.dt = 0.01f;
    in.imu_ok = in.battery_present = in.battery_ok = in.axes_valid = true;
    in.point_count = 1;
    in.nearest_point = 0;
    in.nearest_angle = 0.5f;
    in.nearest_is_edge = true;
    return in;
}

void test_supervisor_arm_balance_fall_fault() {
    Supervisor s;
    SupervisorConfig cfg;
    SupervisorInputs in = ready_inputs();
    for (int i = 0; i < 100; ++i) s.update(in, SupRequest::kNone, cfg);
    TEST_ASSERT_EQUAL(static_cast<int>(SysState::kReady), static_cast<int>(s.state()));
    TEST_ASSERT_EQUAL(static_cast<int>(SupEvent::kArming), static_cast<int>(s.update(in, SupRequest::kArm, cfg)));
    // Not near a point -> stays ARMING, motors off
    for (int i = 0; i < 100; ++i) s.update(in, SupRequest::kNone, cfg);
    TEST_ASSERT_EQUAL(static_cast<int>(SysState::kArming), static_cast<int>(s.state()));
    TEST_ASSERT_FALSE(s.motors_allowed());
    // Near the edge point and still for > 0.3 s -> ARMED_EDGE
    in.nearest_angle = 0.01f;
    for (int i = 0; i < 40; ++i) s.update(in, SupRequest::kNone, cfg);
    TEST_ASSERT_EQUAL(static_cast<int>(SysState::kArmedEdge), static_cast<int>(s.state()));
    TEST_ASSERT_EQUAL(0, s.active_point());
    // Fall
    in.tilt_rad = 0.3f;
    TEST_ASSERT_EQUAL(static_cast<int>(SupEvent::kFell), static_cast<int>(s.update(in, SupRequest::kNone, cfg)));
    TEST_ASSERT_EQUAL(static_cast<int>(SysState::kReady), static_cast<int>(s.state()));
    // Over-current latches even when idle; clear only once gone
    in.tilt_rad = 0;
    in.fault_conditions = fault::kOverCurrent;
    s.update(in, SupRequest::kNone, cfg);
    TEST_ASSERT_EQUAL(static_cast<int>(SysState::kFault), static_cast<int>(s.state()));
    s.update(in, SupRequest::kClearFault, cfg);
    TEST_ASSERT_EQUAL(static_cast<int>(SysState::kFault), static_cast<int>(s.state()));
    in.fault_conditions = 0;
    s.update(in, SupRequest::kClearFault, cfg);
    TEST_ASSERT_EQUAL(static_cast<int>(SysState::kIdle), static_cast<int>(s.state()));
}

void test_supervisor_battery_low_blocks_but_latches_only_when_running() {
    Supervisor s;
    SupervisorConfig cfg;
    SupervisorInputs in = ready_inputs();
    for (int i = 0; i < 100; ++i) s.update(in, SupRequest::kNone, cfg);
    in.battery_ok = false;
    in.fault_conditions = fault::kBatteryLow;
    s.update(in, SupRequest::kNone, cfg);
    TEST_ASSERT_EQUAL(static_cast<int>(SysState::kIdle), static_cast<int>(s.state()));
    TEST_ASSERT_EQUAL_UINT32(0, s.faults());
    TEST_ASSERT_EQUAL(static_cast<int>(SysState::kIdle), static_cast<int>((s.update(in, SupRequest::kArm, cfg), s.state())));
    // Bench with low battery -> fault
    s.update(in, SupRequest::kBenchOn, cfg);
    TEST_ASSERT_EQUAL(static_cast<int>(SysState::kBench), static_cast<int>(s.state()));
    s.update(in, SupRequest::kNone, cfg);
    TEST_ASSERT_EQUAL(static_cast<int>(SysState::kFault), static_cast<int>(s.state()));
    TEST_ASSERT_TRUE(s.faults() & fault::kBatteryLow);
}

void test_supervisor_no_battery_never_ready() {
    Supervisor s;
    SupervisorConfig cfg;
    SupervisorInputs in = ready_inputs();
    in.battery_present = false;
    for (int i = 0; i < 100; ++i) s.update(in, SupRequest::kArm, cfg);
    TEST_ASSERT_EQUAL(static_cast<int>(SysState::kIdle), static_cast<int>(s.state()));
}

// ---------------------------------------------------------------- CLI tokenizer
void test_cli_tokenize() {
    char line[] = "  param  set   e_k1 31.5 \r\n";
    char* argv[8];
    const int argc = cli_tokenize(line, argv, 8);
    TEST_ASSERT_EQUAL(4, argc);
    TEST_ASSERT_EQUAL_STRING("param", argv[0]);
    TEST_ASSERT_EQUAL_STRING("31.5", argv[3]);
    char l2[] = "MOTOR 1 -200";
    TEST_ASSERT_EQUAL(3, cli_tokenize(l2, argv, 8));
    TEST_ASSERT_EQUAL_STRING("motor", argv[0]);  // command word lower-cased
    TEST_ASSERT_TRUE(cli_parse_int("-200", -1000, 1000, nullptr));
    int v = 0;
    TEST_ASSERT_TRUE(cli_parse_int("-200", -1000, 1000, &v));
    TEST_ASSERT_EQUAL(-200, v);
    TEST_ASSERT_FALSE(cli_parse_int("2000", -1000, 1000, &v));
    TEST_ASSERT_FALSE(cli_parse_int("12x", -1000, 1000, &v));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_crc_check_values);
    RUN_TEST(test_spsc_ring_order_and_full);
    RUN_TEST(test_spsc_ring_threads);
    RUN_TEST(test_seqlock_consistent);
    RUN_TEST(test_lowpass_step);
    RUN_TEST(test_parser_v1_and_v2_with_garbage);
    RUN_TEST(test_up_vector_from_quaternion);
    RUN_TEST(test_ina219_math);
    RUN_TEST(test_wheel_speed_constant_rate);
    RUN_TEST(test_params_defaults_in_range_and_roundtrip);
    RUN_TEST(test_calibration_roundtrip_and_points);
    RUN_TEST(test_supervisor_arm_balance_fall_fault);
    RUN_TEST(test_supervisor_battery_low_blocks_but_latches_only_when_running);
    RUN_TEST(test_supervisor_no_battery_never_ready);
    RUN_TEST(test_cli_tokenize);
    return UNITY_END();
}
