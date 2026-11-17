#pragma once
// ============================================================================
// modbus_rtu.hpp - Industrial RS485 Modbus-RTU Master Codec & Noise-Resilient
// Non-Blocking Stream Parser (FC 0x03, 0x04, 0x06 + Exception Frames)
// Author: Le Thanh Danh (Danhle23)
// ============================================================================
#include "iiot/crc16.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace iiot {

enum class FunctionCode : uint8_t {
    ReadHoldingRegisters = 0x03,
    ReadInputRegisters   = 0x04,
    WriteSingleRegister  = 0x06
};

enum class ParseStatus : uint8_t {
    NeedMoreBytes = 0,
    ValidFrame,
    ExceptionFrame,
    CrcMismatch,
    MalformedLength
};

struct ModbusResponse {
    uint8_t                     slave_id{0};
    uint8_t                     function_code{0};
    bool                        is_exception{false};
    uint8_t                     exception_code{0};
    std::size_t                 register_count{0};
    std::array<uint16_t, 64>    registers{};
};

class ModbusRtuCodec {
public:
    // Encode 8-byte request frame [Slave][FC][AddrHi][AddrLo][QtyHi][QtyLo][CRCLo][CRCHi]
    static constexpr std::array<uint8_t, 8> build_read_request(
        uint8_t slave_id,
        FunctionCode fc,
        uint16_t start_addr,
        uint16_t reg_count) noexcept
    {
        std::array<uint8_t, 8> frame{
            slave_id,
            static_cast<uint8_t>(fc),
            static_cast<uint8_t>((start_addr >> 8U) & 0xFFU),
            static_cast<uint8_t>(start_addr & 0xFFU),
            static_cast<uint8_t>((reg_count >> 8U) & 0xFFU),
            static_cast<uint8_t>(reg_count & 0xFFU),
            0U,
            0U
        };
        const uint16_t crc = crc16_modbus(frame.data(), 6);
        frame[6] = static_cast<uint8_t>(crc & 0xFFU);         // Modbus RTU sends CRC Low first
        frame[7] = static_cast<uint8_t>((crc >> 8U) & 0xFFU); // CRC High second
        return frame;
    }

    static constexpr std::array<uint8_t, 8> build_write_single_request(
        uint8_t slave_id,
        uint16_t reg_addr,
        uint16_t reg_value) noexcept
    {
        return build_read_request(slave_id, FunctionCode::WriteSingleRegister, reg_addr, reg_value);
    }

    // Parse response buffer; sets consumed_bytes for sliding-window RS485 noise recovery
    [[nodiscard]] static ParseStatus parse_response(
        const uint8_t* buf,
        std::size_t len,
        ModbusResponse& out,
        std::size_t& consumed_bytes) noexcept
    {
        consumed_bytes = 0;
        if (len < 5) {
            return ParseStatus::NeedMoreBytes;
        }

        const uint8_t slave = buf[0];
        const uint8_t fc    = buf[1];

        // 1. Check Modbus Exception Response (MSB of FC set -> 5-byte frame)
        if ((fc & 0x80U) != 0U) {
            const uint16_t calc_crc = crc16_modbus(buf, 3);
            const uint16_t wire_crc = static_cast<uint16_t>(buf[3]) |
                                      static_cast<uint16_t>(static_cast<uint16_t>(buf[4]) << 8U);
            if (calc_crc != wire_crc) {
                consumed_bytes = 1; // Advance 1 byte to resync over RS485 bus noise
                return ParseStatus::CrcMismatch;
            }
            out.slave_id       = slave;
            out.function_code  = static_cast<uint8_t>(fc & 0x7FU);
            out.is_exception   = true;
            out.exception_code = buf[2];
            out.register_count = 0;
            consumed_bytes     = 5;
            return ParseStatus::ExceptionFrame;
        }

        // 2. Write Single Register Echo (FC 0x06 -> 8-byte frame)
        if (fc == static_cast<uint8_t>(FunctionCode::WriteSingleRegister)) {
            if (len < 8) {
                return ParseStatus::NeedMoreBytes;
            }
            const uint16_t calc_crc = crc16_modbus(buf, 6);
            const uint16_t wire_crc = static_cast<uint16_t>(buf[6]) |
                                      static_cast<uint16_t>(static_cast<uint16_t>(buf[7]) << 8U);
            if (calc_crc != wire_crc) {
                consumed_bytes = 1;
                return ParseStatus::CrcMismatch;
            }
            out.slave_id       = slave;
            out.function_code  = fc;
            out.is_exception   = false;
            out.register_count = 1;
            out.registers[0]   = static_cast<uint16_t>((static_cast<uint16_t>(buf[4]) << 8U) | buf[5]);
            consumed_bytes     = 8;
            return ParseStatus::ValidFrame;
        }

        // 3. Read Holding / Input Registers (FC 0x03 / 0x04)
        if (fc == static_cast<uint8_t>(FunctionCode::ReadHoldingRegisters) ||
            fc == static_cast<uint8_t>(FunctionCode::ReadInputRegisters))
        {
            const uint8_t byte_count = buf[2];
            if ((byte_count == 0U) || ((byte_count & 1U) != 0U) || (byte_count > 128U)) {
                consumed_bytes = 1;
                return ParseStatus::MalformedLength;
            }
            const std::size_t total_frame_len = static_cast<std::size_t>(byte_count) + 5U;
            if (len < total_frame_len) {
                return ParseStatus::NeedMoreBytes;
            }

            const uint16_t calc_crc = crc16_modbus(buf, total_frame_len - 2U);
            const uint16_t wire_crc =
                static_cast<uint16_t>(buf[total_frame_len - 2U]) |
                static_cast<uint16_t>(static_cast<uint16_t>(buf[total_frame_len - 1U]) << 8U);

            if (calc_crc != wire_crc) {
                consumed_bytes = 1;
                return ParseStatus::CrcMismatch;
            }

            out.slave_id       = slave;
            out.function_code  = fc;
            out.is_exception   = false;
            out.exception_code = 0;
            out.register_count = byte_count / 2U;
            for (std::size_t i = 0; i < out.register_count; ++i) {
                const std::size_t hi = 3U + (i * 2U);
                out.registers[i] = static_cast<uint16_t>(
                    (static_cast<uint16_t>(buf[hi]) << 8U) | buf[hi + 1U]);
            }
            consumed_bytes = total_frame_len;
            return ParseStatus::ValidFrame;
        }

        consumed_bytes = 1;
        return ParseStatus::MalformedLength;
    }
};

} // namespace iiot
