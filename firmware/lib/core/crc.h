#pragma once

#include <cstddef>
#include <cstdint>

namespace cubli {

/// CRC-8, poly 0x07, init 0x00, no reflection, no final XOR (IMU PCB frame v1).
uint8_t crc8_07(const uint8_t* data, size_t len);

/// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection (IMU PCB frame v2,
/// parameter blobs). Check value for "123456789" is 0x29B1.
uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF);

}  // namespace cubli
