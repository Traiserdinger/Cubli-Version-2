// Passive buzzer on GPIO19 (Q3 low-side switch): LEDC square wave, duty 50 %.
#pragma once

#include <cstdint>

namespace hal {

class Buzzer {
public:
    void init();
    void tone(uint32_t hz);  // 0 = off
private:
    bool ready_ = false;
    uint32_t hz_ = 0;
};

class StatusLed {
public:
    void init();
    void set(bool on);
};

class BootButton {
public:
    void init();
    bool pressed() const;  // active LOW
};

}  // namespace hal
