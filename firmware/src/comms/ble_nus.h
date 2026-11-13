// BLE console: Nordic UART Service (NUS) on NimBLE. Any "BLE UART" terminal app
// (nRF Toolbox, Serial Bluetooth Terminal, LightBlue, ...) works on Android and iOS.
//   Service 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
//   RX      6E400002-...  (write: commands, one per line)
//   TX      6E400003-...  (notify: replies, events, telemetry)
#pragma once

#include <cstddef>

namespace comms {

inline constexpr size_t kBleLineMax = 96;

bool ble_nus_start(const char* device_name);
bool ble_nus_connected();
/// Blocking send (chunks of MTU-3, retries while the stack is out of buffers).
void ble_nus_send(const char* data, size_t len);
/// Pops one received command line (without CR/LF). Non-blocking.
bool ble_nus_read_line(char* out, size_t cap);

}  // namespace comms
