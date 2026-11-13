// Text output sinks for the console (USB UART0, BLE NUS, or both).
#pragma once

#include <cstdarg>
#include <cstddef>
#include <cstdio>

namespace comms {

class Output {
public:
    virtual ~Output() = default;
    virtual void write(const char* s, size_t n) = 0;
    void print(const char* s);
    void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
};

class UartOutput : public Output {
public:
    void write(const char* s, size_t n) override;
};

class BleOutput : public Output {
public:
    void write(const char* s, size_t n) override;
};

/// Writes to the USB console and, if a client is connected, to BLE.
class BroadcastOutput : public Output {
public:
    void write(const char* s, size_t n) override;
};

UartOutput& uart_out();
BleOutput& ble_out();
BroadcastOutput& all_out();

}  // namespace comms
