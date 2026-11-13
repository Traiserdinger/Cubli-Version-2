// Status LED patterns, buzzer sounds and the BOOT button (core 0, 100 Hz).
//
// Button: hold 1 s   -> arm (READY) / disarm (ARMING or ARMED)
//         one press  -> disarm while armed / arming, otherwise nothing
//         two presses -> store the current pose as a balance point (calib point)
#pragma once

namespace app {
void start_ui_task();
}
