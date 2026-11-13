// Controller, LQR and system-identification tests against the physical simulation.
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include <unity.h>

#include <cmath>
#include <cstdio>

#include "balance.h"
#include "calibration.h"
#include "lqr.h"
#include "params.h"
#include "sim_plant.h"
#include "sysid.h"
#include "wheel_speed.h"

using namespace cubli;

void setUp() {}
void tearDown() {}

namespace {

constexpr float kDt = 0.001f;

LqrWeights weights_from_params(const Params& p) {
    LqrWeights w;
    w.theta_max = p.lqr_th * kDegToRad;
    w.rate_max = p.lqr_rate;
    w.wheel_max = p.lqr_s;
    w.pos_max = p.lqr_p;
    w.u_max = p.lqr_u;
    return w;
}

BalanceConfig config_from(const float ke[4], const float kvx[4], float kv) {
    BalanceConfig c;
    for (int i = 0; i < 4; ++i) {
        c.edge.k[i] = ke[i];
        c.vertex.k[i] = kvx[i];
    }
    c.yaw_k_rate = 0.05f;
    c.yaw_k_wheel = 0.0005f;
    for (float& k : c.kv) k = kv;
    c.u_limit = 1.0f;
    c.pos_limit = 200.0f;
    c.trim_tau_s = 0.0f;
    return c;
}

MotorAxes axes_from(const sim::PlantBase& plant) {
    MotorAxes a;
    for (int k = 0; k < 3; ++k) a.axis[k] = plant.true_axis_imu(k);
    a.finalize();
    return a;
}

struct LoopStats {
    float max_abs_u = 0;
    float max_wheel = 0;
    float final_tilt_deg = 0;
    float max_tilt_deg = 0;
};

template <typename Plant, typename TiltFn>
LoopStats run_loop(Plant& plant, BalanceController& ctrl, const BalanceConfig& cfg, float seconds, TiltFn tilt_deg,
                   SysIdSample* rec = nullptr, int rec_n = 0, float prbs_amp = 0.0f) {
    WheelSpeedEstimator est[3];
    for (int k = 0; k < 3; ++k) est[k].reset(plant.encoder(k), 0u);
    float last_u[3] = {0, 0, 0};
    LoopStats st;
    uint32_t lfsr = 0xACE1u;
    float prbs = prbs_amp;
    const int steps = static_cast<int>(seconds / kDt);
    for (int i = 0; i < steps; ++i) {
        const uint32_t t = static_cast<uint32_t>(i + 1) * 1000u;
        for (int k = 0; k < 3; ++k) plant.u_applied[k] = last_u[k];
        plant.step(kDt);
        BalanceInput in;
        in.up = plant.up_imu();
        in.gyro = plant.gyro_imu();
        in.dt = kDt;
        for (int k = 0; k < 3; ++k) in.wheel[k] = est[k].update(plant.encoder(k), t, 8, 80.0f, 400);
        BalanceOutput out = ctrl.step(in, cfg);
        if (prbs_amp > 0.0f && (i % 20) == 0) {  // PRBS, 50 Hz chip rate
            const uint32_t bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1u;
            lfsr = (lfsr >> 1) | (bit << 15);
            prbs = (lfsr & 1u) ? prbs_amp : -prbs_amp;
        }
        for (int k = 0; k < 3; ++k) {
            float u = out.u[k];
            if (prbs_amp > 0.0f && (ctrl.mode() == BalanceMode::kVertex || k == ctrl.edge_motor())) u += prbs;
            u = clampf(u, -1.0f, 1.0f);
            last_u[k] = u;
            st.max_abs_u = std::fmax(st.max_abs_u, std::fabs(u));
            st.max_wheel = std::fmax(st.max_wheel, std::fabs(in.wheel[k]));
        }
        if (rec && (i % 2) == 0 && (i / 2) < rec_n) {
            SysIdSample& r = rec[i / 2];
            r.dt = 2 * kDt;
            for (int j = 0; j < 2; ++j) {
                r.theta[j] = out.axis[j].theta;
                r.rate[j] = out.axis[j].rate;
            }
            if (ctrl.mode() == BalanceMode::kEdge) {
                r.u[0] = last_u[ctrl.edge_motor()];
                r.wheel_angle[0] = est[ctrl.edge_motor()].angle(400);
            } else {
                Vec3 ang;
                for (int k = 0; k < 3; ++k) ang += plant.true_axis_imu(k) * est[k].angle(400);
                for (int j = 0; j < 2; ++j) r.wheel_angle[j] = dot(ctrl.basis(j), ang);
                // Applied command along each horizontal axis: e_j . sum_k u_k axis_k
                Vec3 v;
                // axes are needed: reconstruct from plant truth
                for (int k = 0; k < 3; ++k) v += plant.true_axis_imu(k) * last_u[k];
                for (int j = 0; j < 2; ++j) r.u[j] = dot(ctrl.basis(j), v);
            }
        }
        const float td = tilt_deg();
        st.max_tilt_deg = std::fmax(st.max_tilt_deg, td);
        st.final_tilt_deg = td;
        if (td > 30.0f) break;
    }
    return st;
}

float edge_gains_lqr(const sim::EdgePlant& pl, float scale_err, float k[4]) {
    Params p;
    params_set_defaults(p);
    AxisModel m{static_cast<float>(pl.a() * scale_err), static_cast<float>(pl.b() / scale_err),
                static_cast<float>(pl.c() * scale_err)};
    const LqrResult r = design_axis_lqr(m, weights_from_params(p), kDt);
    TEST_ASSERT_TRUE_MESSAGE(r.ok, "edge LQR failed");
    for (int i = 0; i < 4; ++i) k[i] = r.k[i];
    return static_cast<float>(r.closed_loop_norm);
}

}  // namespace

void test_lqr_default_model_matches_default_gains() {
    Params p;
    params_set_defaults(p);
    const LqrWeights w = weights_from_params(p);
    const LqrResult e = design_axis_lqr({p.e_a, p.e_b, p.e_c}, w, kDt);
    const LqrResult v = design_axis_lqr({p.v_a, p.v_b, p.v_c}, w, kDt);
    TEST_ASSERT_TRUE(e.ok);
    TEST_ASSERT_TRUE(v.ok);
    char msg[200];
    std::snprintf(msg, sizeof msg, "edge K = %.4g %.4g %.4g %.4g (iter %d) | vertex K = %.4g %.4g %.4g %.4g",
                  static_cast<double>(e.k[0]), static_cast<double>(e.k[1]), static_cast<double>(e.k[2]),
                  static_cast<double>(e.k[3]), e.iterations, static_cast<double>(v.k[0]),
                  static_cast<double>(v.k[1]), static_cast<double>(v.k[2]), static_cast<double>(v.k[3]));
    TEST_MESSAGE(msg);
    // The defaults in params.h must be the LQR solution of the default model.
    const float de[4] = {p.e_k1, p.e_k2, p.e_k3, p.e_k4};
    const float dv[4] = {p.v_k1, p.v_k2, p.v_k3, p.v_k4};
    for (int i = 0; i < 4; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(std::fabs(e.k[i]) * 0.01f + 1e-6f, e.k[i], de[i]);
        TEST_ASSERT_FLOAT_WITHIN(std::fabs(v.k[i]) * 0.01f + 1e-6f, v.k[i], dv[i]);
    }
}

void test_lqr_rejects_bad_model() {
    LqrWeights w;
    TEST_ASSERT_FALSE(design_axis_lqr({-5, 6, 160}, w, kDt).ok);
    TEST_ASSERT_FALSE(design_axis_lqr({60, 0, 160}, w, kDt).ok);
    const float k_zero[4] = {0, 0, 0, 0};
    TEST_ASSERT_FALSE(axis_gain_stabilises({60, 6, 160}, k_zero, kDt));
}

void test_edge_balances_from_3deg() {
    sim::EdgePlant pl;
    pl.theta = 3.0 * M_PI / 180.0;
    float k[4];
    edge_gains_lqr(pl, 1.0f, k);
    const BalanceConfig cfg = config_from(k, k, static_cast<float>(pl.p.kv));
    BalanceController ctrl;
    TEST_ASSERT_TRUE(ctrl.begin_edge(pl.up_eq_imu(), 2, axes_from(pl)));
    const LoopStats st = run_loop(pl, ctrl, cfg, 10.0f, [&] { return static_cast<float>(std::fabs(pl.theta) * 180 / M_PI); });
    char msg[160];
    std::snprintf(msg, sizeof msg, "edge: final tilt %.3f deg, max |u| %.2f, max wheel %.0f rad/s",
                  static_cast<double>(st.final_tilt_deg), static_cast<double>(st.max_abs_u),
                  static_cast<double>(st.max_wheel));
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE(st.final_tilt_deg < 0.3f);
    TEST_ASSERT_TRUE(st.max_wheel < 400.0f);
    TEST_ASSERT_TRUE(std::fabs(pl.s[2]) < 20.0);
}

void test_edge_robust_to_model_error_and_kv_mismatch() {
    for (float err : {0.7f, 1.4f}) {
        sim::EdgePlant pl;
        pl.theta = 2.0 * M_PI / 180.0;
        float k[4];
        edge_gains_lqr(pl, err, k);
        // back-EMF compensation 30 % off as well
        const BalanceConfig cfg = config_from(k, k, static_cast<float>(pl.p.kv * (err > 1 ? 1.3 : 0.7)));
        BalanceController ctrl;
        TEST_ASSERT_TRUE(ctrl.begin_edge(pl.up_eq_imu(), 2, axes_from(pl)));
        const LoopStats st = run_loop(pl, ctrl, cfg, 10.0f, [&] { return static_cast<float>(std::fabs(pl.theta) * 180 / M_PI); });
        TEST_ASSERT_TRUE_MESSAGE(st.final_tilt_deg < 0.5f, "edge unstable with model error");
    }
}

void test_edge_default_gains_balance_nominal_plant() {
    sim::EdgePlant pl;
    pl.theta = 2.0 * M_PI / 180.0;
    Params p;
    params_set_defaults(p);
    const float ke[4] = {p.e_k1, p.e_k2, p.e_k3, p.e_k4};
    const BalanceConfig cfg = config_from(ke, ke, p.m1_kv);
    BalanceController ctrl;
    TEST_ASSERT_TRUE(ctrl.begin_edge(pl.up_eq_imu(), 2, axes_from(pl)));
    const LoopStats st = run_loop(pl, ctrl, cfg, 8.0f, [&] { return static_cast<float>(std::fabs(pl.theta) * 180 / M_PI); });
    TEST_ASSERT_TRUE(st.final_tilt_deg < 0.5f);
}

void test_edge_self_trim_corrects_calibration_error() {
    sim::EdgePlant pl;
    float k[4];
    edge_gains_lqr(pl, 1.0f, k);
    BalanceConfig cfg = config_from(k, k, static_cast<float>(pl.p.kv));
    cfg.trim_tau_s = 5.0f;
    cfg.trim_max_rad = 0.06f;
    // Calibrated point 1.5 deg away from the true one (about the edge axis).
    const double ez[3] = {0, 0, 1};
    double R[3][3];
    sim::axis_angle(ez, 1.5 * M_PI / 180.0, R);
    const Vec3 wrong_eq = sim::rot(pl.mount, sim::rot(R, pl.up_eq_cube()));
    BalanceController ctrl;
    TEST_ASSERT_TRUE(ctrl.begin_edge(wrong_eq, 2, axes_from(pl)));
    run_loop(pl, ctrl, cfg, 40.0f, [&] { return static_cast<float>(std::fabs(pl.theta) * 180 / M_PI); });
    const float residual = angle_between(ctrl.trimmed_equilibrium(), pl.up_eq_imu()) * kRadToDeg;
    char msg[96];
    std::snprintf(msg, sizeof msg, "trim residual %.3f deg, wheel %.1f rad/s", static_cast<double>(residual), pl.s[2]);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE(residual < 0.3f);
    TEST_ASSERT_TRUE(std::fabs(pl.s[2]) < 30.0);
}

void test_vertex_balances_from_3deg() {
    sim::VertexPlant pl;
    pl.set_upright(3.0 * M_PI / 180.0);
    Params p;
    params_set_defaults(p);
    const LqrResult r = design_axis_lqr({static_cast<float>(pl.a()), static_cast<float>(pl.b()), static_cast<float>(pl.c())},
                                        weights_from_params(p), kDt);
    TEST_ASSERT_TRUE(r.ok);
    const BalanceConfig cfg = config_from(r.k, r.k, static_cast<float>(pl.p.kv));
    BalanceController ctrl;
    TEST_ASSERT_TRUE(ctrl.begin_vertex(pl.up_eq_imu(), axes_from(pl)));
    const LoopStats st = run_loop(pl, ctrl, cfg, 10.0f, [&] { return static_cast<float>(pl.tilt() * 180 / M_PI); });
    char msg[160];
    std::snprintf(msg, sizeof msg, "vertex: final tilt %.3f deg, max |u| %.2f, max wheel %.0f rad/s, wheels %.1f %.1f %.1f",
                  static_cast<double>(st.final_tilt_deg), static_cast<double>(st.max_abs_u),
                  static_cast<double>(st.max_wheel), pl.s[0], pl.s[1], pl.s[2]);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE(st.final_tilt_deg < 0.3f);
    for (double s : pl.s) TEST_ASSERT_TRUE(std::fabs(s) < 40.0);
}

void test_vertex_default_gains_balance_nominal_plant() {
    sim::VertexPlant pl;
    pl.set_upright(2.0 * M_PI / 180.0);
    Params p;
    params_set_defaults(p);
    const float kv4[4] = {p.v_k1, p.v_k2, p.v_k3, p.v_k4};
    const BalanceConfig cfg = config_from(kv4, kv4, p.m1_kv);
    BalanceController ctrl;
    TEST_ASSERT_TRUE(ctrl.begin_vertex(pl.up_eq_imu(), axes_from(pl)));
    const LoopStats st = run_loop(pl, ctrl, cfg, 8.0f, [&] { return static_cast<float>(pl.tilt() * 180 / M_PI); });
    TEST_ASSERT_TRUE(st.final_tilt_deg < 0.5f);
}

void test_sysid_edge_recovers_model_in_closed_loop() {
    sim::EdgePlant pl;
    float k[4];
    edge_gains_lqr(pl, 1.3f, k);  // tuned on a wrong model, as before autotune
    const BalanceConfig cfg = config_from(k, k, static_cast<float>(pl.p.kv * 0.8));
    BalanceController ctrl;
    TEST_ASSERT_TRUE(ctrl.begin_edge(pl.up_eq_imu(), 2, axes_from(pl)));
    static SysIdSample rec[1250];
    run_loop(pl, ctrl, cfg, 2.5f, [&] { return static_cast<float>(std::fabs(pl.theta) * 180 / M_PI); }, rec, 1250, 0.08f);
    const SysIdResult r = identify_axis(rec, 1250, 1, 25.0f, -1.0f);
    char msg[200];
    std::snprintf(msg, sizeof msg, "id: a %.1f (%.1f) b %.2f (%.2f) c %.0f (%.0f) kv %.5f (%.5f) R2 %.3f/%.3f %s",
                  static_cast<double>(r.a), pl.a(), static_cast<double>(r.b), pl.b(), static_cast<double>(r.c),
                  pl.c(), static_cast<double>(r.kv), pl.p.kv, static_cast<double>(r.r2_body),
                  static_cast<double>(r.r2_wheel), r.error);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_FLOAT_WITHIN(pl.a() * 0.15, pl.a(), r.a);
    TEST_ASSERT_FLOAT_WITHIN(pl.b() * 0.15, pl.b(), r.b);
    TEST_ASSERT_FLOAT_WITHIN(pl.c() * 0.25, pl.c(), r.c);
}

void test_sysid_vertex_recovers_model() {
    sim::VertexPlant pl;
    pl.set_upright(0.01);
    Params p;
    params_set_defaults(p);
    const LqrResult lq = design_axis_lqr({p.v_a, p.v_b, p.v_c}, weights_from_params(p), kDt);
    const BalanceConfig cfg = config_from(lq.k, lq.k, p.m1_kv);
    BalanceController ctrl;
    TEST_ASSERT_TRUE(ctrl.begin_vertex(pl.up_eq_imu(), axes_from(pl)));
    static SysIdSample rec[1250];
    run_loop(pl, ctrl, cfg, 2.5f, [&] { return static_cast<float>(pl.tilt() * 180 / M_PI); }, rec, 1250, 0.08f);
    const SysIdResult r = identify_axis(rec, 1250, 2, 25.0f, -1.0f);
    char msg[200];
    std::snprintf(msg, sizeof msg, "vertex id: a %.1f (%.1f) b %.2f (%.2f) c %.0f (%.0f) R2 %.3f %s",
                  static_cast<double>(r.a), pl.a(), static_cast<double>(r.b), pl.b(), static_cast<double>(r.c),
                  pl.c(), static_cast<double>(r.r2_body), r.error);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_FLOAT_WITHIN(pl.a() * 0.2, pl.a(), r.a);
    TEST_ASSERT_FLOAT_WITHIN(pl.b() * 0.2, pl.b(), r.b);
}

void test_sysid_motor() {
    // Body fixed: s_dot = c (u - kv s) with friction.
    sim::CubeParams cp;
    static float s[1500], u[1500];
    double sv = 0;
    for (int i = 0; i < 1500; ++i) {
        const double cmd = (i / 250) % 2 ? 0.5 : 0.15;
        for (int j = 0; j < 2; ++j) {
            const double tau = cp.km * (cmd - cp.kv * sv) - cp.friction * sv;
            sv += tau / cp.wheel_I * 0.001;
        }
        s[i] = static_cast<float>(sv);
        u[i] = static_cast<float>(cmd);
    }
    const MotorIdResult r = identify_motor(s, u, 1500, 0.002f, 25.0f);
    TEST_ASSERT_TRUE_MESSAGE(r.ok, r.error);
    const double kv_eff = cp.kv + cp.friction / cp.km;
    TEST_ASSERT_FLOAT_WITHIN(cp.km / cp.wheel_I * 0.05, cp.km / cp.wheel_I, r.c);
    TEST_ASSERT_FLOAT_WITHIN(kv_eff * 0.05, kv_eff, r.kv);
}

void test_axis_calibration_finds_true_axes() {
    sim::VertexPlant pl;
    pl.make_free();  // cube held loosely in the hand
    AxisCalibrator cal;
    cal.start(AxisCalibrator::Config{});
    float u[3];
    for (int i = 0; i < 10000 && cal.status() == AxisCalibrator::Status::kRunning; ++i) {
        cal.step(kDt, pl.gyro_imu(), u);
        for (int k = 0; k < 3; ++k) pl.u_applied[k] = u[k];
        pl.step(kDt);
    }
    TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(AxisCalibrator::Status::kDone), static_cast<int>(cal.status()), cal.error());
    for (int k = 0; k < 3; ++k) {
        const float err = angle_between(cal.result().axis[k], pl.true_axis_imu(k)) * kRadToDeg;
        TEST_ASSERT_TRUE_MESSAGE(err < 8.0f, "axis direction error > 8 deg");
    }
}

void test_point_classification() {
    sim::VertexPlant pl;
    const MotorAxes axes = axes_from(pl);
    int motor = -1;
    TEST_ASSERT_EQUAL(static_cast<int>(PointKind::kVertex), static_cast<int>(classify_point(axes, pl.up_eq_imu(), &motor)));
    sim::EdgePlant pe;
    TEST_ASSERT_EQUAL(static_cast<int>(PointKind::kEdge), static_cast<int>(classify_point(axes_from(pe), pe.up_eq_imu(), &motor)));
    TEST_ASSERT_EQUAL(2, motor);
    // Resting on a face: one axis vertical -> neither.
    TEST_ASSERT_EQUAL(static_cast<int>(PointKind::kUnknown), static_cast<int>(classify_point(axes, axes.axis[0], &motor)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_lqr_default_model_matches_default_gains);
    RUN_TEST(test_lqr_rejects_bad_model);
    RUN_TEST(test_edge_balances_from_3deg);
    RUN_TEST(test_edge_robust_to_model_error_and_kv_mismatch);
    RUN_TEST(test_edge_default_gains_balance_nominal_plant);
    RUN_TEST(test_edge_self_trim_corrects_calibration_error);
    RUN_TEST(test_vertex_balances_from_3deg);
    RUN_TEST(test_vertex_default_gains_balance_nominal_plant);
    RUN_TEST(test_sysid_edge_recovers_model_in_closed_loop);
    RUN_TEST(test_sysid_vertex_recovers_model);
    RUN_TEST(test_sysid_motor);
    RUN_TEST(test_axis_calibration_finds_true_axes);
    RUN_TEST(test_point_classification);
    return UNITY_END();
}
