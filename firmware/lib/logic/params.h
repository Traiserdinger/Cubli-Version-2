// Tunable parameters: one table, stored in NVS, editable from the CLI (USB/BLE).
//
// Every value is stored as float; `kind` tells the CLI how to parse and print it.
// Persisted as (name-hash, value) pairs, so adding or reordering entries keeps
// the other saved values.
#pragma once

#include <cstddef>
#include <cstdint>

namespace cubli {

enum class ParamKind : uint8_t { kFloat, kInt, kBool };

// X(name, kind, default, min, max, help)
// Gains: tau = -(k1*theta[rad] + k2*rate[rad/s] + k3*wheel[rad/s] + k4*wheel_angle[rad]).
// Default gains: LQR (Bryson weights below) on the nominal model e_a..v_c, see
// firmware/README.md "Control design" and tools/lqr_design.py. Replace by autotune.
#define CUBLI_PARAMS(X)                                                                              \
    /* ---- motors (context.md §5) ---- */                                                         \
    X(m1_dir_inv, kBool, 0, 0, 1, "motor 1: invert DIR output")                                    \
    X(m2_dir_inv, kBool, 0, 0, 1, "motor 2: invert DIR output")                                    \
    X(m3_dir_inv, kBool, 0, 0, 1, "motor 3: invert DIR output")                                    \
    X(m1_enc_inv, kBool, 0, 0, 1, "motor 1: invert encoder count")                                 \
    X(m2_enc_inv, kBool, 0, 0, 1, "motor 2: invert encoder count")                                 \
    X(m3_enc_inv, kBool, 0, 0, 1, "motor 3: invert encoder count")                                 \
    X(m1_kv, kFloat, 0.0025f, 0, 0.05f, "motor 1 back-EMF comp [cmd per rad/s]")                    \
    X(m2_kv, kFloat, 0.0025f, 0, 0.05f, "motor 2 back-EMF comp [cmd per rad/s]")                    \
    X(m3_kv, kFloat, 0.0025f, 0, 0.05f, "motor 3 back-EMF comp [cmd per rad/s]")                    \
    X(dir_sw_rads, kFloat, 10000, 0, 10000, "max |wheel speed| for a DIR reversal [rad/s]")         \
    X(u_deadband, kFloat, 0, 0, 0.2f, "command dead-band around 0")                                 \
    X(wheel_win, kInt, 8, 1, 32, "wheel speed differentiation window [samples]")                   \
    X(wheel_fc, kFloat, 80, 0, 500, "wheel speed low-pass cutoff [Hz], 0 = off")                    \
    X(wheel_max, kFloat, 520, 50, 2000, "wheel overspeed fault [rad/s]")                            \
    /* ---- power (context.md §3, §8) ---- */                                                      \
    X(shunt_mohm, kFloat, 50, 1, 200, "INA219 shunt R15 [mOhm]")                                    \
    X(i_range_a, kFloat, 6.0f, 0.5f, 32, "INA219 full-scale current, selects PGA [A]")               \
    X(vbat_scale, kFloat, 1.0f, 0.8f, 1.2f, "battery voltage calibration factor")                   \
    X(i_offset_a, kFloat, 0, -0.5f, 0.5f, "current zero offset [A]")                                \
    X(bat_warn_v, kFloat, 10.5f, 9, 12.6f, "low-battery warning (1 s average) [V]")                 \
    X(bat_cut_v, kFloat, 9.9f, 9, 12.6f, "low-battery cut-off (1 s average) [V]")                   \
    X(i_warn_a, kFloat, 3.5f, 0.5f, 5, "over-current warning [A]")                                  \
    X(i_trip_a, kFloat, 4.5f, 0.5f, 6, "over-current trip [A]")                                     \
    X(i_trip_ms, kInt, 200, 10, 2000, "over-current trip delay [ms]")                               \
    /* ---- balance controller ---- */                                                             \
    X(e_k1, kFloat, 31.55f, -1e4f, 1e4f, "edge gain: tilt")                                          \
    X(e_k2, kFloat, 3.418f, -1e4f, 1e4f, "edge gain: tilt rate")                                     \
    X(e_k3, kFloat, -0.005162f, -1e4f, 1e4f, "edge gain: wheel speed")                                \
    X(e_k4, kFloat, -0.000989f, -1e4f, 1e4f, "edge gain: wheel angle")                                \
    X(v_k1, kFloat, 36.13f, -1e4f, 1e4f, "vertex gain: tilt")                                        \
    X(v_k2, kFloat, 4.224f, -1e4f, 1e4f, "vertex gain: tilt rate")                                   \
    X(v_k3, kFloat, -0.00485f, -1e4f, 1e4f, "vertex gain: wheel speed")                              \
    X(v_k4, kFloat, -0.0009903f, -1e4f, 1e4f, "vertex gain: wheel angle")                              \
    X(y_k1, kFloat, 0.05f, -100, 100, "vertex yaw gain: body spin rate")                            \
    X(y_k2, kFloat, 0.0005f, -1, 1, "vertex yaw gain: wheel momentum")                              \
    X(u_limit, kFloat, 1.0f, 0.05f, 1.0f, "motor command limit")                                    \
    X(p_limit, kFloat, 200, 1, 10000, "wheel-angle state clamp [rad]")                              \
    X(trim_tau, kFloat, 20, 0, 600, "balance-point self-trim time constant [s], 0 = off")          \
    X(trim_max, kFloat, 3, 0, 10, "max self-trim [deg]")                                            \
    /* ---- plant model (autotune writes these) ---- */                                            \
    X(e_a, kFloat, 69.0f, 0, 1000, "edge model: gravity a [1/s^2]")                                 \
    X(e_b, kFloat, 6.7f, 0, 1000, "edge model: motor on body b")                                    \
    X(e_c, kFloat, 167.0f, 0, 10000, "edge model: motor on wheel c")                                \
    X(v_a, kFloat, 62.0f, 0, 1000, "vertex model: gravity a [1/s^2]")                               \
    X(v_b, kFloat, 4.8f, 0, 1000, "vertex model: motor on body b")                                  \
    X(v_c, kFloat, 167.0f, 0, 10000, "vertex model: motor on wheel c")                              \
    /* ---- LQR weights (Bryson: largest acceptable value of each state) ---- */                   \
    X(lqr_th, kFloat, 2.0f, 0.1f, 20, "LQR: tilt [deg]")                                            \
    X(lqr_rate, kFloat, 0.5f, 0.01f, 20, "LQR: tilt rate [rad/s]")                                  \
    X(lqr_s, kFloat, 400, 1, 5000, "LQR: wheel speed [rad/s]")                                      \
    X(lqr_p, kFloat, 600, 1, 100000, "LQR: wheel angle [rad]")                                      \
    X(lqr_u, kFloat, 0.6f, 0.01f, 1, "LQR: motor command")                                          \
    /* ---- arming and safety ---- */                                                              \
    X(mode, kInt, 0, 0, 2, "balance points allowed: 0 auto, 1 edge only, 2 vertex only")           \
    X(arm_cap, kFloat, 2.0f, 0.2f, 10, "capture: max tilt from a calibrated point [deg]")          \
    X(arm_rate, kFloat, 0.3f, 0.01f, 5, "capture: max body rate [rad/s]")                           \
    X(arm_hold, kFloat, 0.3f, 0, 5, "capture: hold time [s]")                                       \
    X(arm_tout, kFloat, 30, 1, 600, "ARMING timeout [s]")                                           \
    X(tilt_lim, kFloat, 10, 1, 30, "fall detection: tilt from balance point [deg]")                \
    X(rearm, kBool, 0, 0, 1, "after a fall, go back to ARMING instead of READY")                   \
    X(brake_rads, kFloat, 60, 0, 2000, "on stop: brake at once below this wheel speed [rad/s]")     \
    X(coast_s, kFloat, 1.5f, 0, 10, "on stop: max coast time before braking [s]")                   \
    X(imu_tout, kInt, 10, 3, 200, "IMU frame timeout while armed [ms]")                             \
    /* ---- autotune / system identification ---- */                                               \
    X(at_amp, kFloat, 0.08f, 0, 0.5f, "excitation amplitude (PRBS) [cmd]")                          \
    X(at_time, kFloat, 2.5f, 0.5f, 3.0f, "recording length [s]")                                    \
    X(at_fc, kFloat, 25, 1, 200, "identification filter cutoff [Hz]")                               \
    /* ---- telemetry ---- */                                                                      \
    X(tlm_hz, kInt, 0, 0, 500, "USB telemetry rate [Hz], 0 = off")                                  \
    X(ble_tlm_hz, kInt, 0, 0, 50, "BLE telemetry rate [Hz], 0 = off")

struct Params {
#define CUBLI_PARAM_FIELD(name, kind, def, lo, hi, help) float name;
    CUBLI_PARAMS(CUBLI_PARAM_FIELD)
#undef CUBLI_PARAM_FIELD

    float dir_inv(int k) const { return k == 0 ? m1_dir_inv : (k == 1 ? m2_dir_inv : m3_dir_inv); }
    float enc_inv(int k) const { return k == 0 ? m1_enc_inv : (k == 1 ? m2_enc_inv : m3_enc_inv); }
    float kv(int k) const { return k == 0 ? m1_kv : (k == 1 ? m2_kv : m3_kv); }
    float* kv_ptr(int k) { return k == 0 ? &m1_kv : (k == 1 ? &m2_kv : &m3_kv); }
    float* dir_inv_ptr(int k) { return k == 0 ? &m1_dir_inv : (k == 1 ? &m2_dir_inv : &m3_dir_inv); }
    float* enc_inv_ptr(int k) { return k == 0 ? &m1_enc_inv : (k == 1 ? &m2_enc_inv : &m3_enc_inv); }
};

struct ParamInfo {
    const char* name;
    ParamKind kind;
    float def, min, max;
    const char* help;
    size_t offset;
};

size_t param_count();
const ParamInfo& param_info(size_t i);
/// Index of a parameter by name, or -1.
int param_find(const char* name);

void params_set_defaults(Params& p);
float param_get(const Params& p, size_t i);
/// Validates range and integer-ness. Returns nullptr on success, else an error text.
const char* param_set(Params& p, size_t i, float value);
/// Parses text ("1.5", "on", "true", ...) and sets. Returns nullptr on success.
const char* param_set_text(Params& p, const char* name, const char* text);
/// Formats the value of parameter i into buf.
void param_format(const Params& p, size_t i, char* buf, size_t len);

// ---- persistence ----
inline constexpr uint32_t kParamBlobMagic = 0x43504231;  // "CPB1"
/// Bytes needed by params_serialize.
size_t params_blob_size();
size_t params_serialize(const Params& p, uint8_t* out, size_t cap);
/// Starts from defaults and applies every recognised stored value. Returns the
/// number of values applied, or -1 if the blob is corrupt (params = defaults).
int params_deserialize(Params& p, const uint8_t* in, size_t len);

}  // namespace cubli
