#include "app/ui_task.h"

#include "app/shared.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/buzzer.h"

namespace app {
namespace {

using cubli::SupEvent;
using cubli::SysState;

struct Note {
    uint16_t hz;
    uint16_t ms;
};

// Melodies (0 Hz = rest). Terminated by {0, 0}.
constexpr Note kBoot[] = {{2000, 80}, {0, 40}, {2700, 80}, {0, 40}, {3400, 120}, {0, 0}};
constexpr Note kReady[] = {{2700, 60}, {0, 0}};
constexpr Note kArming[] = {{2000, 60}, {0, 60}, {2000, 60}, {0, 0}};
constexpr Note kArmed[] = {{3400, 150}, {0, 0}};
constexpr Note kDisarmed[] = {{2700, 80}, {0, 40}, {2000, 120}, {0, 0}};
constexpr Note kFell[] = {{1500, 250}, {0, 0}};
constexpr Note kFault[] = {{1200, 150}, {0, 80}, {1200, 150}, {0, 80}, {1200, 300}, {0, 0}};
constexpr Note kClick[] = {{3000, 25}, {0, 0}};
constexpr Note kLowBattery[] = {{1800, 100}, {0, 100}, {1800, 100}, {0, 0}};

class Player {
public:
    void play(const Note* m) {
        melody_ = m;
        idx_ = 0;
        left_ms_ = 0;
    }
    void tick(hal::Buzzer& bz, int dt_ms) {
        if (!melody_) {
            bz.tone(0);
            return;
        }
        left_ms_ -= dt_ms;
        if (left_ms_ > 0) return;
        const Note& n = melody_[idx_];
        if (n.hz == 0 && n.ms == 0) {
            melody_ = nullptr;
            bz.tone(0);
            return;
        }
        bz.tone(n.hz);
        left_ms_ = n.ms;
        ++idx_;
    }

private:
    const Note* melody_ = nullptr;
    int idx_ = 0;
    int left_ms_ = 0;
};

/// LED pattern: period and on-time in ms (on >= period -> solid).
void led_pattern(SysState s, int& period, int& on, int& flashes) {
    flashes = 1;
    switch (s) {
        case SysState::kBoot: period = 100; on = 50; break;
        case SysState::kIdle: period = 2000; on = 60; break;
        case SysState::kReady: period = 2000; on = 60; flashes = 2; break;
        case SysState::kCalibrate: period = 500; on = 250; break;
        case SysState::kArming: period = 250; on = 125; break;
        case SysState::kArmedEdge:
        case SysState::kArmedVertex: period = 1000; on = 1000; break;
        case SysState::kBench: period = 1000; on = 500; break;
        case SysState::kFault: period = 100; on = 50; break;
    }
}

void ui_task(void*) {
    hal::Buzzer bz;
    hal::StatusLed led;
    hal::BootButton btn;
    bz.init();
    led.init();
    btn.init();
    Player player;
    player.play(kBoot);

    constexpr int kDtMs = 10;
    int t_ms = 0;
    bool prev = btn.pressed();  // ignore a press held through boot
    bool armed_from_hold = false;
    int press_ms = 0, release_gap_ms = 10000, clicks = 0;
    int low_batt_ms = 0;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(kDtMs));
        t_ms += kDtMs;

        ControlStatus st;
        g_status.read(st);

        // ---- supervisor events -> sounds ----
        ControlEvent ev;
        while (g_ui_events.pop(ev)) {
            switch (static_cast<SupEvent>(ev.i0)) {
                case SupEvent::kReady: player.play(kReady); break;
                case SupEvent::kArming: player.play(kArming); break;
                case SupEvent::kArmed: player.play(kArmed); break;
                case SupEvent::kDisarmed:
                case SupEvent::kArmTimeout: player.play(kDisarmed); break;
                case SupEvent::kFell: player.play(kFell); break;
                case SupEvent::kFault: player.play(kFault); break;
                default: break;
            }
        }

        if (g_beep_request.exchange(false)) player.play(kArmed);

        // ---- low battery reminder every 10 s ----
        PowerStatus pw;
        g_power.read(pw);
        if (pw.low_warning || (pw.present && !pw.above_cutoff && pw.samples > 60)) {
            low_batt_ms += kDtMs;
            if (low_batt_ms >= 10000) {
                low_batt_ms = 0;
                player.play(kLowBattery);
            }
        } else {
            low_batt_ms = 0;
        }
        player.tick(bz, kDtMs);

        // ---- LED ----
        int period = 1000, on = 0, flashes = 1;
        led_pattern(st.state, period, on, flashes);
        bool led_on;
        if (on >= period) {
            led_on = true;
        } else if (flashes == 2) {
            const int ph = t_ms % period;
            led_on = ph < on || (ph >= 2 * on && ph < 3 * on);
        } else {
            led_on = (t_ms % period) < on;
        }
        led.set(led_on);

        // ---- button ----
        const bool now = btn.pressed();
        if (now) {
            press_ms += kDtMs;
            if (press_ms >= 1000 && !armed_from_hold) {
                armed_from_hold = true;
                clicks = 0;
                player.play(kClick);
                const bool active = st.state == SysState::kArming || st.state == SysState::kArmedEdge ||
                                    st.state == SysState::kArmedVertex;
                send_command(active ? Cmd::kDisarm : Cmd::kArm);
            }
        } else {
            if (prev && !armed_from_hold && press_ms >= 30) {
                const bool active = st.state == SysState::kArming || st.state == SysState::kArmedEdge ||
                                    st.state == SysState::kArmedVertex;
                if (active) {
                    send_command(Cmd::kDisarm);
                    clicks = 0;
                } else {
                    clicks = release_gap_ms < 400 ? clicks + 1 : 1;
                    release_gap_ms = 0;
                    if (clicks >= 2) {
                        clicks = 0;
                        player.play(kClick);
                        send_command(Cmd::kCalibPoint);
                    }
                }
            }
            if (prev) {
                press_ms = 0;
                armed_from_hold = false;
            }
            release_gap_ms += kDtMs;
        }
        prev = now;
    }
}

}  // namespace

void start_ui_task() { xTaskCreatePinnedToCore(ui_task, "ui", 3072, nullptr, 3, nullptr, 0); }

}  // namespace app
