#include "crc.h"

namespace cubli {

uint8_t crc8_07(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80u) ? static_cast<uint8_t>((crc << 1) ^ 0x07u) : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc) {
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u) : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

}  // namespace cubli
