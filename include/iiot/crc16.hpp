#pragma once
// ============================================================================
// crc16.hpp - Deterministic Zero-Heap CRC-16/MODBUS (Polynomial 0xA001)
// Compile-time constexpr lookup table for O(1) per-byte MCU execution
// Author: Le Thanh Danh (Danhle23)
// ============================================================================
#include <array>
#include <cstddef>
#include <cstdint>

namespace iiot {

namespace detail {
constexpr std::array<uint16_t, 256> make_modbus_crc_table() noexcept {
    std::array<uint16_t, 256> table{};
    for (uint16_t i = 0; i < 256; ++i) {
        uint16_t crc = i;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x0001U) {
                crc = static_cast<uint16_t>((crc >> 1U) ^ 0xA001U);
            } else {
                crc = static_cast<uint16_t>(crc >> 1U);
            }
        }
        table[i] = crc;
    }
    return table;
}

inline constexpr auto kCrc16Table = make_modbus_crc_table();
} // namespace detail

[[nodiscard]] constexpr uint16_t crc16_modbus(const uint8_t* data, std::size_t len) noexcept {
    uint16_t crc = 0xFFFFU;
    for (std::size_t i = 0; i < len; ++i) {
        const uint8_t idx = static_cast<uint8_t>((crc ^ data[i]) & 0xFFU);
        crc = static_cast<uint16_t>((crc >> 8U) ^ detail::kCrc16Table[idx]);
    }
    return crc;
}

} // namespace iiot
