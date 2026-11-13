#include "hal/ina219_dev.h"

#include "board_pins.h"
#include "driver/i2c_master.h"
#include "hw_config.h"
#include "ina219_math.h"

namespace hal {
namespace {
constexpr int kTimeoutMs = 5;
}

bool Ina219::init() {
    i2c_master_bus_config_t bc = {};
    bc.i2c_port = I2C_NUM_0;
    bc.sda_io_num = static_cast<gpio_num_t>(board::kPinI2cSda);
    bc.scl_io_num = static_cast<gpio_num_t>(board::kPinI2cScl);
    bc.clk_source = I2C_CLK_SRC_DEFAULT;
    bc.glitch_ignore_cnt = 7;
    bc.flags.enable_internal_pullup = false;  // R9/R17 4k7 on the board
    i2c_master_bus_handle_t bus = nullptr;
    if (i2c_new_master_bus(&bc, &bus) != ESP_OK) return false;
    bus_ = bus;
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address = board::kIna219Address;
    dc.scl_speed_hz = hw::kI2cClockHz;
    i2c_master_dev_handle_t dev = nullptr;
    if (i2c_master_bus_add_device(bus, &dc, &dev) != ESP_OK) return false;
    dev_ = dev;
    present_ = write_reg(cubli::ina219::kRegConfig, cubli::ina219::kConfigReset);
    return present_;
}

bool Ina219::configure(float shunt_ohm, float max_current_a) {
    shunt_ohm_ = shunt_ohm;
    const uint8_t pga = cubli::ina219::select_pga(max_current_a, shunt_ohm);
    const uint16_t cfg = cubli::ina219::make_config(pga);
    uint16_t back = 0;
    present_ = write_reg(cubli::ina219::kRegConfig, cfg) && read_reg(cubli::ina219::kRegConfig, &back) && back == cfg;
    return present_;
}

bool Ina219::read(float* volts, float* amps, bool* overflow) {
    uint16_t bus = 0, shunt = 0;
    if (!read_reg(cubli::ina219::kRegBus, &bus) || !read_reg(cubli::ina219::kRegShunt, &shunt)) {
        present_ = false;
        return false;
    }
    const auto b = cubli::ina219::decode_bus(bus);
    *volts = b.volts;
    *amps = cubli::ina219::decode_current(shunt, shunt_ohm_);
    *overflow = b.overflow;
    present_ = true;
    return true;
}

bool Ina219::write_reg(uint8_t reg, uint16_t v) {
    if (!dev_) return false;
    const uint8_t buf[3] = {reg, static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v & 0xFF)};
    return i2c_master_transmit(static_cast<i2c_master_dev_handle_t>(dev_), buf, sizeof buf, kTimeoutMs) == ESP_OK;
}

bool Ina219::read_reg(uint8_t reg, uint16_t* v) {
    if (!dev_) return false;
    uint8_t rx[2] = {0, 0};
    if (i2c_master_transmit_receive(static_cast<i2c_master_dev_handle_t>(dev_), &reg, 1, rx, 2, kTimeoutMs) != ESP_OK)
        return false;
    *v = static_cast<uint16_t>((rx[0] << 8) | rx[1]);
    return true;
}

}  // namespace hal
