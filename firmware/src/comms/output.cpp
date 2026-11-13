#include "comms/output.h"

#include <cstring>

#include "comms/ble_nus.h"
#include "driver/uart.h"

namespace comms {

void Output::print(const char* s) { write(s, std::strlen(s)); }

void Output::printf(const char* fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    const int n = std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) write(buf, static_cast<size_t>(n) < sizeof buf ? static_cast<size_t>(n) : sizeof buf - 1);
}

void UartOutput::write(const char* s, size_t n) { uart_write_bytes(UART_NUM_0, s, n); }

void BleOutput::write(const char* s, size_t n) { ble_nus_send(s, n); }

void BroadcastOutput::write(const char* s, size_t n) {
    uart_out().write(s, n);
    if (ble_nus_connected()) ble_out().write(s, n);
}

UartOutput& uart_out() {
    static UartOutput o;
    return o;
}
BleOutput& ble_out() {
    static BleOutput o;
    return o;
}
BroadcastOutput& all_out() {
    static BroadcastOutput o;
    return o;
}

}  // namespace comms
