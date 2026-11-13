// Console task (core 0): command line over USB (UART0) and BLE, control-task
// event reporting, NVS saving, and the off-line part of autotune (system
// identification + LQR design never run on the control core).
#pragma once

namespace app {
void start_console_task();
void start_telemetry_task();
}
