// INA219 on I2C0 (GPIO21/22, 400 kHz). Register arithmetic is in lib/estimation.
#pragma once

#include <cstdint>

namespace hal {

class Ina219 {
public:
    bool init();
    /// (Re)writes the configuration for a shunt value and full-scale current.
    bool configure(float shunt_ohm, float max_current_a);
    /// Reads bus voltage and current. Returns false on an I2C error.
    bool read(float* volts, float* amps, bool* overflow);
    bool present() const { return present_; }

private:
    bool write_reg(uint8_t reg, uint16_t v);
    bool read_reg(uint8_t reg, uint16_t* v);

    void* bus_ = nullptr;
    void* dev_ = nullptr;
    bool present_ = false;
    float shunt_ohm_ = 0.05f;
};

}  // namespace hal
