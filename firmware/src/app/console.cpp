#include "app/console.h"

#include <cmath>
#include <cstring>

#include "app/shared.h"
#include "board_pins.h"
#include "cli_parse.h"
#include "comms/ble_nus.h"
#include "comms/output.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/imu_link.h"
#include "hal/nvs_store.h"
#include "hw_config.h"
#include "imu_frame.h"
#include "lqr.h"
#include "sysid.h"

namespace app {
namespace {

using comms::Output;
using cubli::SysState;

constexpr const char* kVersion = "CubliV2 firmware 0.1.0 (" __DATE__ ")";

// ------------------------------------------------------------------ helpers
ControlStatus status() {
    ControlStatus s;
    g_status.read(s);
    return s;
}

bool motors_running() { return g_motors_live.load(); }

const char* kind_name(cubli::PointKind k) {
    switch (k) {
        case cubli::PointKind::kEdge: return "edge";
        case cubli::PointKind::kVertex: return "vertex";
        default: return "unknown";
    }
}

const char* procedure_name(Procedure p) {
    switch (p) {
        case Procedure::kNone: return "none";
        case Procedure::kAxes: return "calib axes";
        case Procedure::kSysIdMotor: return "sysid motor";
        case Procedure::kAutotune: return "autotune";
        case Procedure::kAutotuneDrop: return "autotune drop";
        case Procedure::kLog: return "log";
        case Procedure::kWireTest: return "wiretest";
    }
    return "?";
}

cubli::LqrWeights weights(const cubli::Params& p) {
    cubli::LqrWeights w;
    w.theta_max = p.lqr_th * cubli::kDegToRad;
    w.rate_max = p.lqr_rate;
    w.wheel_max = p.lqr_s;
    w.pos_max = p.lqr_p;
    w.u_max = p.lqr_u;
    return w;
}

// Pending result of autotune / sysid, applied by "autotune apply".
struct Pending {
    enum class Kind { kNone, kEdge, kVertex, kMotor } kind = Kind::kNone;
    int motor = -1;
    cubli::AxisModel model;
    float k[4] = {0, 0, 0, 0};
    float kv = 0;
} g_pending;

bool g_calib_save_pending = false;

// ------------------------------------------------------------------ commands
struct Ctx {
    Output& out;
};

using Handler = void (*)(Ctx&, int, char**);
struct Command {
    const char* name;
    Handler fn;
    const char* usage;
    const char* help;
};

void need_bench(Ctx& c) {
    const ControlStatus s = status();
    if (s.state != SysState::kBench) {
        c.out.print("! not in BENCH: run 'bench on' first (battery required, motors on the bench)\r\n");
    }
}

void cmd_status(Ctx& c, int, char**) {
    const ControlStatus s = status();
    PowerStatus pw;
    g_power.read(pw);
    char faults[96], conds[96];
    c.out.printf("state %s  faults [%s]  conditions [%s]\r\n", cubli::sys_state_name(s.state),
                 cubli::fault_names(s.faults, faults, sizeof faults),
                 cubli::fault_names(s.fault_conditions, conds, sizeof conds));
    if (s.state == SysState::kIdle) c.out.printf("not ready: %s\r\n", s.not_ready);
    c.out.printf("battery %.2f V (avg %.2f)  %.2f A  %s%s\r\n", static_cast<double>(pw.v_inst),
                 static_cast<double>(pw.v_avg), static_cast<double>(pw.i_inst),
                 pw.sensor_ok ? (pw.present ? "present" : "absent (USB only)") : "INA219 not responding",
                 pw.low_warning ? " LOW" : "");
    c.out.printf("imu %s @%lu  ekf %s  age %.1f ms  frames v2 %lu  crc err %lu  gaps %lu\r\n",
                 hal::imu_link_state_name(static_cast<hal::ImuLink::State>(s.imu_state)),
                 static_cast<unsigned long>(s.imu_baud), (s.imu_status & cubli::imu_status::kEkfReady) ? "ready" : "not ready",
                 static_cast<double>(s.imu_age_ms), static_cast<unsigned long>(s.imu_frames_v2),
                 static_cast<unsigned long>(s.imu_crc_errors), static_cast<unsigned long>(s.imu_gaps));
    if (s.nearest_point >= 0)
        c.out.printf("nearest balance point #%d at %.2f deg\r\n", s.nearest_point, static_cast<double>(s.nearest_deg));
    if (s.active_point >= 0)
        c.out.printf("balancing on point #%d  tilt %.2f deg\r\n", s.active_point, static_cast<double>(s.tilt_deg));
    c.out.printf("wheels %.0f %.0f %.0f rad/s  cmd %.2f %.2f %.2f  brake %s\r\n", static_cast<double>(s.wheel[0]),
                 static_cast<double>(s.wheel[1]), static_cast<double>(s.wheel[2]), static_cast<double>(s.u[0]),
                 static_cast<double>(s.u[1]), static_cast<double>(s.u[2]), s.brake_released ? "released" : "ON");
    c.out.printf("loop %.0f Hz  exec max %lu us  period %lu..%lu us  overruns %lu%s\r\n",
                 static_cast<double>(s.loop_hz), static_cast<unsigned long>(s.exec_us_max),
                 static_cast<unsigned long>(s.period_us_min), static_cast<unsigned long>(s.period_us_max),
                 static_cast<unsigned long>(s.overruns), s.deadman_tripped ? "  DEADMAN TRIPPED" : "");
    if (s.procedure) c.out.printf("procedure: %s (%.1f s)\r\n", procedure_name(static_cast<Procedure>(s.procedure)),
                                  static_cast<double>(s.procedure_progress));
}

void cmd_bat(Ctx& c, int, char**) {
    PowerStatus pw;
    g_power.read(pw);
    const cubli::Params p = g_params.get();
    c.out.printf("V %.3f (1 s avg %.3f)  I %.3f A (avg %.3f)  P %.2f W\r\n", static_cast<double>(pw.v_inst),
                 static_cast<double>(pw.v_avg), static_cast<double>(pw.i_inst), static_cast<double>(pw.i_avg),
                 static_cast<double>(pw.power_w));
    c.out.printf("sensor %s  present %d  above cut-off %d  low %d  I warn %d  over-current %d  shunt %.1f mOhm%s\r\n",
                 pw.sensor_ok ? "ok" : "ERROR", pw.present, pw.above_cutoff, pw.low_warning, pw.current_warning,
                 pw.overcurrent, static_cast<double>(p.shunt_mohm), pw.shunt_overflow ? "  SHUNT OVERFLOW" : "");
    c.out.printf("thresholds: warn %.2f V, cut-off %.2f V, I warn %.1f A, trip %.1f A / %d ms\r\n",
                 static_cast<double>(p.bat_warn_v), static_cast<double>(p.bat_cut_v), static_cast<double>(p.i_warn_a),
                 static_cast<double>(p.i_trip_a), static_cast<int>(p.i_trip_ms));
}

void cmd_enc(Ctx& c, int argc, char** argv) {
    if (argc > 1 && !std::strcmp(argv[1], "zero")) {
        send_command(Cmd::kEncZero);
        c.out.print("encoder counts zeroed\r\n");
        return;
    }
    const ControlStatus s = status();
    for (int k = 0; k < 3; ++k) {
        c.out.printf("motor %d (%s/%s): count %ld (raw %ld)  %.2f rev  speed %.1f rad/s (%.0f rpm)\r\n", k + 1,
                     board::kMotorConnector[k], board::kMotorSilkscreen[k], static_cast<long>(s.enc_rel[k]),
                     static_cast<long>(s.enc_raw[k]),
                     static_cast<double>(static_cast<float>(s.enc_rel[k]) / hw::kEncoderCountsPerRev),
                     static_cast<double>(s.wheel[k]), static_cast<double>(s.wheel[k] * 60.0f / (2.0f * cubli::kPi)));
    }
}

void cmd_imu(Ctx& c, int argc, char** argv) {
    if (argc > 1 && !std::strcmp(argv[1], "send") && argc > 2) {
        ControlCommand cc;
        cc.cmd = Cmd::kImuSend;
        std::strncpy(cc.text, argv[2], sizeof(cc.text) - 1);
        send_command(cc);
        c.out.printf("sent '%s' to the IMU PCB\r\n", cc.text);
        return;
    }
    if (argc > 1 && !std::strcmp(argv[1], "restart")) {
        send_command(Cmd::kImuRestart);
        c.out.print("IMU link: searching again\r\n");
        return;
    }
    const ControlStatus s = status();
    c.out.printf("link %s @%lu baud  status 0x%02X (ekf %d imu1 %d imu2 %d accel-rejected %d mag %d)\r\n",
                 hal::imu_link_state_name(static_cast<hal::ImuLink::State>(s.imu_state)),
                 static_cast<unsigned long>(s.imu_baud), s.imu_status, !!(s.imu_status & cubli::imu_status::kEkfReady),
                 !!(s.imu_status & cubli::imu_status::kImu1Ok), !!(s.imu_status & cubli::imu_status::kImu2Ok),
                 !!(s.imu_status & cubli::imu_status::kAccelRejected), !!(s.imu_status & cubli::imu_status::kMagFused));
    c.out.printf("frames v1 %lu v2 %lu  crc errors %lu  seq gaps %lu  uart overflows %lu  age %.1f ms\r\n",
                 static_cast<unsigned long>(s.imu_frames_v1), static_cast<unsigned long>(s.imu_frames_v2),
                 static_cast<unsigned long>(s.imu_crc_errors), static_cast<unsigned long>(s.imu_gaps),
                 static_cast<unsigned long>(s.imu_overflows), static_cast<double>(s.imu_age_ms));
    c.out.printf("q %.4f %.4f %.4f %.4f  up %.4f %.4f %.4f\r\n", static_cast<double>(s.q.w), static_cast<double>(s.q.x),
                 static_cast<double>(s.q.y), static_cast<double>(s.q.z), static_cast<double>(s.up.x),
                 static_cast<double>(s.up.y), static_cast<double>(s.up.z));
    c.out.printf("gyro %.3f %.3f %.3f rad/s  accel %.3f %.3f %.3f g\r\n", static_cast<double>(s.gyro.x),
                 static_cast<double>(s.gyro.y), static_cast<double>(s.gyro.z), static_cast<double>(s.accel.x),
                 static_cast<double>(s.accel.y), static_cast<double>(s.accel.z));
    if (s.imu_frames_v1)
        c.out.printf("v1: roll %.2f pitch %.2f yaw %.2f deg  IMU PCB supply %.2f V\r\n", static_cast<double>(s.v1_roll),
                     static_cast<double>(s.v1_pitch), static_cast<double>(s.v1_yaw), static_cast<double>(s.imu_supply_v));
}

void cmd_bench(Ctx& c, int argc, char** argv) {
    if (argc < 2) {
        c.out.print("usage: bench on|off\r\n");
        return;
    }
    const bool on = !std::strcmp(argv[1], "on");
    send_command(on ? Cmd::kBenchOn : Cmd::kBenchOff);
    c.out.print(on ? "BENCH: brake engaged; 'brake off', then 'motor <n> <cmd>' (auto-stop after 3 s)\r\n"
                   : "bench off\r\n");
}

void cmd_brake(Ctx& c, int argc, char** argv) {
    if (argc < 2 || (std::strcmp(argv[1], "on") && std::strcmp(argv[1], "off"))) {
        c.out.print("usage: brake on|off\r\n");
        return;
    }
    need_bench(c);
    send_command(Cmd::kBrake, !std::strcmp(argv[1], "on") ? 1 : 0);
}

void cmd_motor(Ctx& c, int argc, char** argv) {
    int n = 0, v = 0;
    if (argc < 3 || !cubli::cli_parse_int(argv[1], 1, 3, &n) || !cubli::cli_parse_int(argv[2], -1000, 1000, &v)) {
        c.out.print("usage: motor <1-3> <-1000..1000>   (per-mille of full command)\r\n");
        return;
    }
    need_bench(c);
    send_command(Cmd::kMotor, n - 1, static_cast<float>(v) / 1000.0f);
    c.out.printf("motor %d (%s / silkscreen %s) -> %d/1000 for 3 s\r\n", n, board::kMotorConnector[n - 1],
                 board::kMotorSilkscreen[n - 1], v);
}

void cmd_stop(Ctx& c, int, char**) {
    send_command(Cmd::kStop);
    c.out.print("stop: all motor commands zero, disarmed\r\n");
}

void cmd_beep(Ctx& c, int, char**) {
    g_beep_request.store(true);
    c.out.print("beep\r\n");
}

void cmd_wiretest(Ctx& c, int argc, char** argv) {
    int n = 0;
    if (argc < 2 || !cubli::cli_parse_int(argv[1], 1, 3, &n)) {
        c.out.print("usage: wiretest <1-3>\r\n");
        return;
    }
    const int k = n - 1;
    c.out.printf(
        "WIRE TEST motor %d = schematic Motor %d = connector %s = silkscreen %s (%s edge)\r\n"
        "Expected wiring (context.md 5.1): 1 red +12V, 2 black GND, 3 yellow BRAKE, 4 white PWM (inverted),\r\n"
        "  5 green DIR, 6 blue +5V encoder, 7 purple CHB, 8 grey CHA. Pin 1 = square pad.\r\n"
        "Step A (by hand, no battery needed): 'enc zero', turn the wheel exactly 10 turns, 'enc'.\r\n"
        "  Expect |count| = 4000 (400 CPR). 0 -> encoder supply/signals; ~2000 -> one channel missing.\r\n"
        "Step B (automatic, battery, wheel free): +15%% 1 s, coast, -15%% 1 s, coast, +15%% with BRAKE on.\r\n",
        n, n, board::kMotorConnector[k], board::kMotorSilkscreen[k], board::kMotorBoardEdge[k]);
    const ControlStatus s = status();
    if (s.state != SysState::kBench) {
        send_command(Cmd::kBenchOn);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    if (status().state != SysState::kBench) {
        c.out.print("! cannot enter BENCH (battery connected? fault?) - step B skipped\r\n");
        return;
    }
    send_command(Cmd::kWireTest, k);
    c.out.print("running step B (4.6 s)...\r\n");
}

void cmd_param(Ctx& c, int argc, char** argv) {
    const char* sub = argc > 1 ? argv[1] : "list";
    char buf[32];
    if (!std::strcmp(sub, "list")) {
        const cubli::Params p = g_params.get();
        const char* filt = argc > 2 ? argv[2] : nullptr;
        for (size_t i = 0; i < cubli::param_count(); ++i) {
            const cubli::ParamInfo& pi = cubli::param_info(i);
            if (filt && !std::strstr(pi.name, filt)) continue;
            cubli::param_format(p, i, buf, sizeof buf);
            c.out.printf("%-12s %-12s %s\r\n", pi.name, buf, pi.help);
        }
    } else if (!std::strcmp(sub, "get") && argc > 2) {
        const int i = cubli::param_find(argv[2]);
        if (i < 0) {
            c.out.print("unknown parameter\r\n");
            return;
        }
        const cubli::Params p = g_params.get();
        cubli::param_format(p, static_cast<size_t>(i), buf, sizeof buf);
        const cubli::ParamInfo& pi = cubli::param_info(static_cast<size_t>(i));
        c.out.printf("%s = %s  (default %g, range %g..%g) %s\r\n", pi.name, buf, static_cast<double>(pi.def),
                     static_cast<double>(pi.min), static_cast<double>(pi.max), pi.help);
    } else if (!std::strcmp(sub, "set") && argc > 3) {
        const char* err = nullptr;
        g_params.update([&](cubli::Params& p) {
            err = cubli::param_set_text(p, argv[2], argv[3]);
            return err == nullptr;
        });
        if (err) c.out.printf("! %s\r\n", err);
        else c.out.printf("%s = %s (active now; 'param save' to keep)\r\n", argv[2], argv[3]);
    } else if (!std::strcmp(sub, "save")) {
        if (motors_running()) {
            c.out.print("! motors running: saving would stall the control loop. Disarm first.\r\n");
            return;
        }
        c.out.print(hal::nvs_save_params(g_params.get()) ? "parameters saved\r\n" : "! NVS write failed\r\n");
    } else if (!std::strcmp(sub, "reset")) {
        g_params.update([](cubli::Params& p) {
            cubli::params_set_defaults(p);
            return true;
        });
        c.out.print("defaults loaded (not saved)\r\n");
    } else if (!std::strcmp(sub, "load")) {
        cubli::Params p;
        const int n = hal::nvs_load_params(p);
        g_params.update([&](cubli::Params& q) {
            q = p;
            return true;
        });
        c.out.printf("loaded %d stored values\r\n", n);
    } else {
        c.out.print("usage: param list [filter] | get <name> | set <name> <value> | save | load | reset\r\n");
    }
}

void print_calib(Ctx& c) {
    cubli::CalibrationData cal;
    g_calib.read(cal);
    c.out.printf("motor axes %s\r\n", cal.axes_valid ? "calibrated" : "NOT calibrated (calib axes)");
    for (int k = 0; k < 3; ++k)
        c.out.printf("  axis %d: %+.3f %+.3f %+.3f\r\n", k + 1, static_cast<double>(cal.axis[k].x),
                     static_cast<double>(cal.axis[k].y), static_cast<double>(cal.axis[k].z));
    for (int i = 0; i < cubli::kMaxBalancePoints; ++i) {
        const cubli::BalancePoint& pt = cal.points[i];
        if (!pt.valid) continue;
        c.out.printf("  point #%d %-7s motor %d  up %+.4f %+.4f %+.4f\r\n", i, kind_name(pt.kind), pt.motor + 1,
                     static_cast<double>(pt.up.x), static_cast<double>(pt.up.y), static_cast<double>(pt.up.z));
    }
}

void cmd_calib(Ctx& c, int argc, char** argv) {
    const char* sub = argc > 1 ? argv[1] : "show";
    if (!std::strcmp(sub, "show")) {
        print_calib(c);
    } else if (!std::strcmp(sub, "axes")) {
        c.out.print(
            "AXIS CALIBRATION: hold the cube in the air between fingertips, loosely, so it can twist.\r\n"
            "Each motor gets a short kick (wheel jerks). Keep still between kicks. Takes ~3 s. Battery required.\r\n");
        send_command(Cmd::kCalibAxes);
    } else if (!std::strcmp(sub, "point")) {
        c.out.print("hold the cube exactly at the balance point (edge or vertex), still, for 0.5 s...\r\n");
        send_command(Cmd::kCalibPoint);
    } else if (!std::strcmp(sub, "delete") && argc > 2) {
        int i = 0;
        if (!cubli::cli_parse_int(argv[2], 0, cubli::kMaxBalancePoints - 1, &i)) {
            c.out.print("usage: calib delete <0-7>\r\n");
            return;
        }
        send_command(Cmd::kCalibDeletePoint, i);
    } else if (!std::strcmp(sub, "clear")) {
        const bool all = argc > 2 && !std::strcmp(argv[2], "all");
        send_command(Cmd::kCalibClear, all ? 1 : 0);
        c.out.print(all ? "axes and balance points cleared\r\n" : "balance points cleared\r\n");
    } else if (!std::strcmp(sub, "gyro")) {
        ControlCommand cc;
        cc.cmd = Cmd::kImuSend;
        std::strncpy(cc.text, "REINIT", sizeof(cc.text) - 1);
        send_command(cc);
        c.out.print("IMU PCB re-initialises its EKF and gyro bias: keep the cube completely still for 2 s\r\n");
    } else if (!std::strcmp(sub, "bat") && argc > 2) {
        float vm = 0;
        PowerStatus pw;
        g_power.read(pw);
        if (!cubli::cli_parse_float(argv[2], 5.0f, 20.0f, &vm) || !(pw.v_avg > 5.0f)) {
            c.out.print("usage: calib bat <measured pack volts>   (battery connected)\r\n");
            return;
        }
        g_params.update([&](cubli::Params& p) {
            const float raw = pw.v_avg / p.vbat_scale;
            return cubli::param_set(p, static_cast<size_t>(cubli::param_find("vbat_scale")), vm / raw) == nullptr;
        });
        c.out.print("vbat_scale updated ('param save' to keep)\r\n");
    } else if (!std::strcmp(sub, "current")) {
        PowerStatus pw;
        g_power.read(pw);
        g_params.update([&](cubli::Params& p) {
            const float raw = pw.i_avg + p.i_offset_a;
            return cubli::param_set(p, static_cast<size_t>(cubli::param_find("i_offset_a")), raw) == nullptr;
        });
        c.out.print("current zero offset updated (motors must be stopped) - 'param save' to keep\r\n");
    } else if (!std::strcmp(sub, "axis") && argc > 5) {
        int n = 0;
        float v[3];
        if (!cubli::cli_parse_int(argv[2], 1, 3, &n) || !cubli::cli_parse_float(argv[3], -1, 1, &v[0]) ||
            !cubli::cli_parse_float(argv[4], -1, 1, &v[1]) || !cubli::cli_parse_float(argv[5], -1, 1, &v[2])) {
            c.out.print("usage: calib axis <1-3> <x> <y> <z>\r\n");
            return;
        }
        cubli::CalibrationData cal;
        g_calib.read(cal);
        cal.axis[n - 1] = cubli::Vec3{v[0], v[1], v[2]};
        cubli::MotorAxes a;
        for (int k = 0; k < 3; ++k) a.axis[k] = cal.axis[k];
        cal.axes_valid = a.finalize() ? 1 : 0;
        g_calib_in.write(cal);
        send_command(Cmd::kSetCalib);
        c.out.printf("axis %d set; axes %s\r\n", n, cal.axes_valid ? "valid" : "not orthogonal yet");
    } else {
        c.out.print(
            "usage: calib show | axes | point | delete <i> | clear [all] | gyro | bat <volts> | current |"
            " axis <n> <x> <y> <z>\r\n");
    }
}

void cmd_arm(Ctx& c, int, char**) {
    const ControlStatus s = status();
    if (s.state != SysState::kReady) {
        c.out.printf("! not READY (%s): %s\r\n", cubli::sys_state_name(s.state), s.not_ready);
        return;
    }
    send_command(Cmd::kArm);
    c.out.print("ARMING: place the cube on a calibrated edge/vertex and hold it still; it starts by itself\r\n");
}

void cmd_disarm(Ctx& c, int, char**) {
    send_command(Cmd::kDisarm);
    c.out.print("disarm\r\n");
}

void cmd_fault(Ctx& c, int argc, char** argv) {
    if (argc > 1 && !std::strcmp(argv[1], "clear")) {
        send_command(Cmd::kClearFault);
        c.out.print("fault clear requested\r\n");
        return;
    }
    cmd_status(c, 0, nullptr);
}

void cmd_mode(Ctx& c, int argc, char** argv) {
    int m = -1;
    if (argc > 1) {
        if (!std::strcmp(argv[1], "auto")) m = 0;
        if (!std::strcmp(argv[1], "edge")) m = 1;
        if (!std::strcmp(argv[1], "vertex")) m = 2;
    }
    if (m < 0) {
        c.out.print("usage: mode auto|edge|vertex\r\n");
        return;
    }
    g_params.update([&](cubli::Params& p) {
        p.mode = static_cast<float>(m);
        return true;
    });
    c.out.printf("mode = %s\r\n", argv[1]);
}

void print_model_and_gains(Output& out, const char* what, const cubli::AxisModel& m, const float k[4]) {
    out.printf("%s model: a %.2f  b %.3f  c %.1f\r\n", what, static_cast<double>(m.a), static_cast<double>(m.b),
                 static_cast<double>(m.c));
    out.printf("%s LQR gains: k1 %.5g  k2 %.5g  k3 %.5g  k4 %.5g\r\n", what, static_cast<double>(k[0]),
                 static_cast<double>(k[1]), static_cast<double>(k[2]), static_cast<double>(k[3]));
}

void cmd_lqr(Ctx& c, int argc, char** argv) {
    const bool vertex = argc > 1 && !std::strcmp(argv[1], "vertex");
    const bool apply = (argc > 1 && !std::strcmp(argv[argc - 1], "apply"));
    const cubli::Params p = g_params.get();
    const cubli::AxisModel m = vertex ? cubli::AxisModel{p.v_a, p.v_b, p.v_c} : cubli::AxisModel{p.e_a, p.e_b, p.e_c};
    const cubli::LqrResult r = cubli::design_axis_lqr(m, weights(p), 0.001f);
    if (!r.ok) {
        c.out.print("! LQR failed: model not plausible (a, b, c must be > 0)\r\n");
        return;
    }
    print_model_and_gains(c.out, vertex ? "vertex" : "edge", m, r.k);
    if (apply) {
        g_params.update([&](cubli::Params& q) {
            float* dst[4] = {vertex ? &q.v_k1 : &q.e_k1, vertex ? &q.v_k2 : &q.e_k2, vertex ? &q.v_k3 : &q.e_k3,
                             vertex ? &q.v_k4 : &q.e_k4};
            for (int i = 0; i < 4; ++i) *dst[i] = r.k[i];
            return true;
        });
        c.out.print("gains applied (active now; 'param save' to keep)\r\n");
    } else {
        c.out.printf("'lqr %s apply' writes these gains\r\n", vertex ? "vertex" : "edge");
    }
}

void cmd_autotune(Ctx& c, int argc, char** argv) {
    const char* sub = argc > 1 ? argv[1] : "";
    const ControlStatus s = status();
    if (!std::strcmp(sub, "run") || !std::strcmp(sub, "edge") || !std::strcmp(sub, "vertex")) {
        if (s.state != SysState::kArmedEdge && s.state != SysState::kArmedVertex) {
            c.out.print(
                "! autotune runs while balancing: arm first (current gains must hold the cube),\r\n"
                "  or use 'autotune drop' (edge, open loop) if the cube cannot balance yet\r\n");
            return;
        }
        send_command(Cmd::kAutotune, 0);
        c.out.print("recording 2.5 s with excitation while balancing - keep hands off...\r\n");
    } else if (!std::strcmp(sub, "drop")) {
        if (s.state != SysState::kReady) {
            c.out.print("! needs READY\r\n");
            return;
        }
        send_command(Cmd::kAutotune, 1);
        c.out.print(
            "DROP TEST: put the cube on a calibrated EDGE, hold it at the balance point and let go when it arms;\r\n"
            "the wheel is excited open-loop while it falls (max 8 deg / 0.8 s), then it stops. Soft surface!\r\n");
    } else if (!std::strcmp(sub, "apply")) {
        if (g_pending.kind == Pending::Kind::kNone) {
            c.out.print("! no result pending\r\n");
            return;
        }
        const Pending pd = g_pending;
        g_params.update([&](cubli::Params& p) {
            if (pd.kind == Pending::Kind::kMotor) {
                *p.kv_ptr(pd.motor) = pd.kv;
            } else if (pd.kind == Pending::Kind::kEdge) {
                p.e_a = pd.model.a; p.e_b = pd.model.b; p.e_c = pd.model.c;
                p.e_k1 = pd.k[0]; p.e_k2 = pd.k[1]; p.e_k3 = pd.k[2]; p.e_k4 = pd.k[3];
            } else {
                p.v_a = pd.model.a; p.v_b = pd.model.b; p.v_c = pd.model.c;
                p.v_k1 = pd.k[0]; p.v_k2 = pd.k[1]; p.v_k3 = pd.k[2]; p.v_k4 = pd.k[3];
            }
            return true;
        });
        g_pending.kind = Pending::Kind::kNone;
        c.out.print("applied (active now; 'param save' to keep)\r\n");
    } else if (!std::strcmp(sub, "cancel")) {
        send_command(Cmd::kAutotuneCancel);
        g_pending.kind = Pending::Kind::kNone;
        c.out.print("cancelled\r\n");
    } else {
        c.out.print(
            "usage: autotune run | drop | apply | cancel\r\n"
            "  1. 'sysid motor <n>' for each motor (cube on a face) -> back-EMF kv\r\n"
            "  2. arm on an edge, 'autotune run' -> model a,b,c + LQR gains; 'autotune apply'\r\n"
            "  3. same on the vertex; then 'param save'\r\n");
    }
}

void cmd_sysid(Ctx& c, int argc, char** argv) {
    int n = 0;
    if (argc < 3 || std::strcmp(argv[1], "motor") || !cubli::cli_parse_int(argv[2], 1, 3, &n)) {
        c.out.print("usage: sysid motor <1-3>   (cube resting on a face, wheel free, battery)\r\n");
        return;
    }
    send_command(Cmd::kSysIdMotor, n - 1);
    c.out.printf("motor %d staircase 2.5 s...\r\n", n);
}

void cmd_log(Ctx& c, int argc, char** argv) {
    int ms = 2000;
    if (argc > 1 && !cubli::cli_parse_int(argv[1], 100, 2500, &ms)) {
        c.out.print("usage: log [100..2500 ms]\r\n");
        return;
    }
    send_command(Cmd::kLog, ms);
    c.out.printf("recording %d ms at 500 Hz...\r\n", ms);
}

void cmd_tlm(Ctx& c, int argc, char** argv) {
    const bool ble = argc > 1 && !std::strcmp(argv[1], "ble");
    const char* v = argc > (ble ? 2 : 1) ? argv[ble ? 2 : 1] : nullptr;
    int hz = 0;
    if (!v || (std::strcmp(v, "off") && !cubli::cli_parse_int(v, 0, ble ? 50 : 500, &hz))) {
        c.out.print("usage: tlm <0..500>|off  |  tlm ble <0..50>|off\r\n"
                    "format: $T,t_ms,state,brake,th0_deg,th1_deg,rate0,rate1,w1,w2,w3,u1,u2,u3,V,A,exec_us\r\n");
        return;
    }
    g_params.update([&](cubli::Params& p) {
        (ble ? p.ble_tlm_hz : p.tlm_hz) = static_cast<float>(hz);
        return true;
    });
    c.out.printf("%s telemetry %d Hz\r\n", ble ? "BLE" : "USB", hz);
}

void cmd_reboot(Ctx& c, int, char**) {
    if (motors_running()) {
        c.out.print("! disarm first\r\n");
        return;
    }
    c.out.print("rebooting\r\n");
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
}

void cmd_version(Ctx& c, int, char**) { c.out.printf("%s\r\n", kVersion); }
void cmd_help(Ctx& c, int, char**);

const Command kCommands[] = {
    {"help", cmd_help, "", "this list"},
    {"version", cmd_version, "", "firmware version"},
    {"status", cmd_status, "", "state, faults, battery, IMU, loop timing"},
    {"bat", cmd_bat, "", "battery voltage / current / thresholds"},
    {"enc", cmd_enc, "[zero]", "encoder counts and wheel speeds"},
    {"imu", cmd_imu, "[send <cmd>|restart]", "IMU PCB link and attitude"},
    {"bench", cmd_bench, "on|off", "bench mode (manual motor commands)"},
    {"brake", cmd_brake, "on|off", "shared BRAKE line (bench)"},
    {"motor", cmd_motor, "<1-3> <-1000..1000>", "motor command per mille (bench, 3 s)"},
    {"stop", cmd_stop, "", "stop all motors, disarm"},
    {"beep", cmd_beep, "", "buzzer test"},
    {"wiretest", cmd_wiretest, "<1-3>", "guided Nidec 24H wiring check"},
    {"param", cmd_param, "list|get|set|save|load|reset", "parameters (NVS)"},
    {"calib", cmd_calib, "show|axes|point|delete|clear|gyro|bat|current|axis", "calibration"},
    {"arm", cmd_arm, "", "start balancing (hold BOOT 1 s does the same)"},
    {"disarm", cmd_disarm, "", "stop balancing"},
    {"fault", cmd_fault, "[clear]", "show / clear latched faults"},
    {"mode", cmd_mode, "auto|edge|vertex", "which balance points may arm"},
    {"lqr", cmd_lqr, "[edge|vertex] [apply]", "LQR gains from the stored model"},
    {"autotune", cmd_autotune, "run|drop|apply|cancel", "identify the model and compute optimal gains"},
    {"sysid", cmd_sysid, "motor <1-3>", "identify motor back-EMF (cube on a face)"},
    {"log", cmd_log, "[ms]", "record 500 Hz control data and dump CSV"},
    {"tlm", cmd_tlm, "[ble] <hz>|off", "telemetry stream"},
    {"reboot", cmd_reboot, "", "restart"},
};

void cmd_help(Ctx& c, int, char**) {
    for (const Command& cmd : kCommands) c.out.printf("  %-9s %-32s %s\r\n", cmd.name, cmd.usage, cmd.help);
}

void execute(char* line, Output& out) {
    char* argv[10];
    const int argc = cubli::cli_tokenize(line, argv, 10);
    if (argc == 0) return;
    Ctx ctx{out};
    for (const Command& cmd : kCommands) {
        if (!std::strcmp(cmd.name, argv[0])) {
            cmd.fn(ctx, argc, argv);
            return;
        }
    }
    out.printf("unknown command '%s' - type 'help'\r\n", argv[0]);
}

// ------------------------------------------------------------------ events
void report_identification(Output& out) {
    Recording& r = g_recording;
    const cubli::Params p = g_params.get();
    if (r.procedure == Procedure::kLog) {
        out.print("$L,i,dt,theta0,theta1,rate0,rate1,wheel_angle0,wheel_angle1,u0,u1\r\n");
        for (int i = 0; i < r.count; ++i) {
            const cubli::SysIdSample& s = r.samples[i];
            out.printf("$L,%d,%.4f,%.5f,%.5f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f\r\n", i, static_cast<double>(s.dt),
                       static_cast<double>(s.theta[0]), static_cast<double>(s.theta[1]),
                       static_cast<double>(s.rate[0]), static_cast<double>(s.rate[1]),
                       static_cast<double>(s.wheel_angle[0]), static_cast<double>(s.wheel_angle[1]),
                       static_cast<double>(s.u[0]), static_cast<double>(s.u[1]));
            if ((i & 31) == 0) vTaskDelay(1);
        }
        out.print("$L,end\r\n");
        return;
    }
    if (r.procedure == Procedure::kSysIdMotor) {
        static float* speed = new float[cubli::kSysIdMaxSamples];  // once, outside the control loop
        static float* cmd = new float[cubli::kSysIdMaxSamples];
        const int n = r.count;
        for (int i = 0; i < n; ++i) {
            const int a = i > 0 ? i - 1 : 0, b = i < n - 1 ? i + 1 : n - 1;
            speed[i] = (r.samples[b].wheel_angle[0] - r.samples[a].wheel_angle[0]) / (0.002f * static_cast<float>(b - a));
            cmd[i] = r.samples[i].u[0];
        }
        const cubli::MotorIdResult m = cubli::identify_motor(speed, cmd, n, 0.002f, p.at_fc);
        out.printf("sysid motor %d: c %.1f  kv %.5f (no-load speed at full command %.0f rad/s)  R2 %.3f  %s\r\n",
                   r.motor + 1, static_cast<double>(m.c), static_cast<double>(m.kv),
                   m.kv > 0 ? 1.0 / static_cast<double>(m.kv) : 0.0, static_cast<double>(m.r2),
                   m.ok ? "OK" : m.error);
        if (m.ok) {
            g_pending = Pending{};
            g_pending.kind = Pending::Kind::kMotor;
            g_pending.motor = r.motor;
            g_pending.kv = m.kv;
            out.printf("'autotune apply' sets m%d_kv = %.5f\r\n", r.motor + 1, static_cast<double>(m.kv));
        }
        return;
    }
    // autotune (closed loop or drop test)
    const bool vertex = r.naxes == 2;
    const cubli::SysIdResult id = cubli::identify_axis(r.samples, r.count, r.naxes, p.at_fc, r.kv_mean);
    out.printf("identified (%d samples): a %.2f  b %.3f  c %.1f  kv %.5f  R2 body %.3f wheel %.3f  %s\r\n", id.samples,
               static_cast<double>(id.a), static_cast<double>(id.b), static_cast<double>(id.c),
               static_cast<double>(id.kv), static_cast<double>(id.r2_body), static_cast<double>(id.r2_wheel),
               id.ok ? "OK" : id.error);
    if (!id.ok) return;
    const cubli::AxisModel m{id.a, id.b, id.c};
    const float cur[4] = {vertex ? p.v_k1 : p.e_k1, vertex ? p.v_k2 : p.e_k2, vertex ? p.v_k3 : p.e_k3,
                          vertex ? p.v_k4 : p.e_k4};
    out.printf("current gains on the identified model: %s\r\n",
               cubli::axis_gain_stabilises(m, cur, 0.001f) ? "stable" : "NOT stable");
    const cubli::LqrResult lq = cubli::design_axis_lqr(m, weights(p), 0.001f);
    if (!lq.ok) {
        out.print("! LQR design failed\r\n");
        return;
    }
    print_model_and_gains(out, vertex ? "vertex" : "edge", m, lq.k);
    g_pending = Pending{};
    g_pending.kind = vertex ? Pending::Kind::kVertex : Pending::Kind::kEdge;
    g_pending.model = m;
    for (int i = 0; i < 4; ++i) g_pending.k[i] = lq.k[i];
    out.print("'autotune apply' writes model + gains (then 'param save')\r\n");
}

void handle_event(const ControlEvent& e, Output& out) {
    switch (e.ev) {
        case Ev::kSupervisor: {
            const auto ev = static_cast<cubli::SupEvent>(e.i0);
            const char* what = nullptr;
            switch (ev) {
                case cubli::SupEvent::kReady: what = "READY"; break;
                case cubli::SupEvent::kNotReady: what = "not ready"; break;
                case cubli::SupEvent::kArming: what = "ARMING"; break;
                case cubli::SupEvent::kArmed: what = "ARMED - balancing"; break;
                case cubli::SupEvent::kDisarmed: what = "disarmed"; break;
                case cubli::SupEvent::kFell: what = "fell (tilt limit) - disarmed"; break;
                case cubli::SupEvent::kArmTimeout: what = "ARMING timed out"; break;
                case cubli::SupEvent::kFault: what = "FAULT"; break;
                case cubli::SupEvent::kFaultCleared: what = "fault cleared"; break;
                default: break;
            }
            if (what) {
                out.printf("[%s] %s\r\n", cubli::sys_state_name(static_cast<SysState>(e.i1)), what);
                if (ev == cubli::SupEvent::kFault) {
                    const ControlStatus s = status();
                    char buf[96];
                    out.printf("  faults: %s ('fault clear' once the cause is gone)\r\n",
                               cubli::fault_names(s.faults, buf, sizeof buf));
                }
            }
            break;
        }
        case Ev::kWireTestDone: {
            const int k = e.i0;
            const float fwd = e.f[0], rev = e.f[1], braked = e.f[2];
            out.printf("WIRE TEST motor %d: counts +cmd %.0f, -cmd %.0f, +cmd braked %.0f, speed %.1f rad/s\r\n", k + 1,
                       static_cast<double>(fwd), static_cast<double>(rev), static_cast<double>(braked),
                       static_cast<double>(e.f[3]));
            const bool pwm_ok = std::fabs(fwd) > 40.0f;
            const bool dir_ok = pwm_ok && std::fabs(rev) > 40.0f && (fwd > 0) != (rev > 0);
            const bool brake_ok = std::fabs(braked) < 0.25f * std::fabs(fwd);
            out.printf("  PWM (white, inverted) %s\r\n", pwm_ok ? "OK" : "FAIL: wheel did not turn (PWM, +12 V, BRAKE?)");
            out.printf("  DIR (green)           %s\r\n", dir_ok ? "OK" : "FAIL: no reversal");
            out.printf("  BRAKE (yellow)        %s\r\n", brake_ok ? "OK" : "FAIL: wheel turned with BRAKE low");
            if (pwm_ok) {
                const cubli::Params p = g_params.get();
                const bool inv_now = p.enc_inv(k) > 0.5f;
                if (fwd < 0)
                    out.printf("  -> positive command counted NEGATIVE: 'param set m%d_enc_inv %d', 'param save'\r\n", k + 1,
                               inv_now ? 0 : 1);
                else
                    out.printf("  -> encoder sign OK (m%d_enc_inv = %d)\r\n", k + 1, inv_now ? 1 : 0);
                out.printf("  m%d_dir_inv only chooses which way is 'positive'; calib axes adapts to either\r\n", k + 1);
            }
            out.printf("  mapping: schematic Motor %d = %s = silkscreen %s; cube axis from 'calib axes'\r\n", k + 1,
                       board::kMotorConnector[k], board::kMotorSilkscreen[k]);
            break;
        }
        case Ev::kAxesDone:
            out.printf("motor axes calibrated (body rate change %.2f %.2f %.2f rad/s)\r\n", static_cast<double>(e.f[0]),
                       static_cast<double>(e.f[1]), static_cast<double>(e.f[2]));
            break;
        case Ev::kAxesFailed: {
            out.printf("! axis calibration failed at motor %d\r\n", static_cast<int>(e.i0) + 1);
            break;
        }
        case Ev::kPointStored:
            out.printf("balance point #%d stored: %s%s%d\r\n", static_cast<int>(e.i0), kind_name(static_cast<cubli::PointKind>(e.i1)),
                       e.i1 == static_cast<int>(cubli::PointKind::kEdge) ? ", motor " : "",
                       e.i1 == static_cast<int>(cubli::PointKind::kEdge) ? static_cast<int>(e.f[0]) + 1 : 0);
            if (e.i1 == static_cast<int>(cubli::PointKind::kUnknown))
                out.print("  (kind unknown: calibrate the axes, or the pose is not an edge/vertex)\r\n");
            break;
        case Ev::kPointRejected: {
            static const char* const kWhy[] = {"IMU not ready", "motors running", "cube moved during capture",
                                               "all 8 slots used (calib delete <i>)"};
            out.printf("! balance point not stored: %s\r\n", kWhy[e.i0 < 4 ? e.i0 : 0]);
            break;
        }
        case Ev::kRecordDone: report_identification(out); release_recording(); break;
        case Ev::kImuLinkChanged:
            out.printf("IMU link: %s\r\n", hal::imu_link_state_name(static_cast<hal::ImuLink::State>(e.i0)));
            if (static_cast<hal::ImuLink::State>(e.i0) == hal::ImuLink::State::kV1)
                out.print("  frame v1 only so far: balancing needs the CubliV2 IMU PCB patch (imu_pcb/)\r\n");
            break;
        case Ev::kCalibChanged: g_calib_save_pending = true; break;
        case Ev::kBenchTimeout: out.print("bench: motor command timed out (3 s) - stopped\r\n"); break;
    }
}

// ------------------------------------------------------------------ console task
class LineEditor {
public:
    /// Returns true when a full line is available in line().
    bool feed(char ch, Output& echo) {
        if (ch == '\r' || ch == '\n') {
            if (len_ == 0) return false;
            buf_[len_] = '\0';
            len_ = 0;
            echo.print("\r\n");
            return true;
        }
        if ((ch == '\b' || ch == 0x7F) && len_ > 0) {
            --len_;
            echo.print("\b \b");
            return false;
        }
        if (ch >= 0x20 && ch < 0x7F && len_ < sizeof(buf_) - 1) {
            buf_[len_++] = ch;
            echo.write(&ch, 1);
        }
        return false;
    }
    char* line() { return buf_; }

private:
    char buf_[128] = {};
    size_t len_ = 0;
};

void console_task(void*) {
    comms::Output& usb = comms::uart_out();
    usb.printf("\r\n%s\r\ntype 'help'. Motors are braked until armed.\r\n", kVersion);
    const uint32_t rr = g_reset_reason.load();
    if (rr == ESP_RST_TASK_WDT || rr == ESP_RST_INT_WDT || rr == ESP_RST_WDT || rr == ESP_RST_PANIC)
        usb.printf("! previous reset by watchdog/panic (reason %lu)\r\n", static_cast<unsigned long>(rr));
    LineEditor ed;
    char ble_line[comms::kBleLineMax];
    for (;;) {
        uint8_t buf[64];
        const int n = uart_read_bytes(UART_NUM_0, buf, sizeof buf, pdMS_TO_TICKS(10));
        for (int i = 0; i < n; ++i) {
            if (ed.feed(static_cast<char>(buf[i]), usb)) execute(ed.line(), usb);
        }
        while (comms::ble_nus_read_line(ble_line, sizeof ble_line)) {
            comms::ble_out().printf("> %s\r\n", ble_line);
            execute(ble_line, comms::ble_out());
        }
        ControlEvent ev;
        while (g_events.pop(ev)) handle_event(ev, comms::all_out());
        if (g_calib_save_pending && !motors_running()) {
            cubli::CalibrationData cal;
            if (g_calib.read(cal)) {
                g_calib_save_pending = false;
                comms::all_out().print(hal::nvs_save_calib(cal) ? "calibration saved\r\n" : "! calibration save failed\r\n");
            }
        }
    }
}

// ------------------------------------------------------------------ telemetry task
void telemetry_task(void*) {
    uint32_t n = 0;
    for (;;) {
        TelemetrySample t;
        if (!g_telemetry.pop(t)) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        const cubli::Params p = g_params.get();
        char line[200];
        const int len = std::snprintf(
            line, sizeof line, "$T,%lu,%u,%u,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.2f,%.2f,%u\r\n",
            static_cast<unsigned long>(t.t_ms), t.state, t.brake, static_cast<double>(t.theta[0] * cubli::kRadToDeg),
            static_cast<double>(t.theta[1] * cubli::kRadToDeg), static_cast<double>(t.rate[0]),
            static_cast<double>(t.rate[1]), static_cast<double>(t.wheel[0]), static_cast<double>(t.wheel[1]),
            static_cast<double>(t.wheel[2]), static_cast<double>(t.u[0]), static_cast<double>(t.u[1]),
            static_cast<double>(t.u[2]), static_cast<double>(t.volts), static_cast<double>(t.amps), t.exec_us);
        if (len <= 0) continue;
        const int rate = static_cast<int>(std::fmax(p.tlm_hz, p.ble_tlm_hz));
        if (p.tlm_hz > 0) comms::uart_out().write(line, static_cast<size_t>(len));
        if (p.ble_tlm_hz > 0 && comms::ble_nus_connected() && rate > 0) {
            const int div = rate / static_cast<int>(p.ble_tlm_hz) > 0 ? rate / static_cast<int>(p.ble_tlm_hz) : 1;
            if ((n % static_cast<uint32_t>(div)) == 0) comms::ble_out().write(line, static_cast<size_t>(len));
        }
        ++n;
    }
}

}  // namespace

void start_console_task() {
    uart_driver_install(UART_NUM_0, 512, 4096, 0, nullptr, 0);
    xTaskCreatePinnedToCore(console_task, "console", 8192, nullptr, 4, nullptr, 0);
}

void start_telemetry_task() { xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, nullptr, 2, nullptr, 0); }

}  // namespace app
