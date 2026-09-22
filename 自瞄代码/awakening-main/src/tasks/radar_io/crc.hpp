#pragma once

#include <cstddef>
#include <cstdint>

namespace awakening::radar_io {

uint8_t get_crc8(const uint8_t* data, uint32_t len);
bool verify_crc8(const uint8_t* data, uint32_t len);

uint16_t get_crc16(const uint8_t* data, uint32_t len);
bool verify_crc16(const uint8_t* data, uint32_t len);

} // namespace awakening::radar_io