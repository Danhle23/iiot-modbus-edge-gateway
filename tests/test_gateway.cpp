// ============================================================================
// test_gateway.cpp - Automated C++17 Verification Suite for IIoT Edge Gateway
// Tests CRC-16/MODBUS, Lock-Free SPSC RingBuffer, Noise-Resilient Modbus-RTU
// Framing, Exception Decoding, and Debounced 3-Phase / Cold-Chain Faults.
// Author: Le Thanh Danh (Danhle23)
// ============================================================================
#include "iiot/crc16.hpp"
#include "iiot/fault_monitor.hpp"
#include "iiot/modbus_rtu.hpp"
#include "iiot/ring_buffer.hpp"

#include <cstdio>
#include <cstring>
#include <vector>

namespace {
int g_passed = 0;
int g_failed = 0;

void check(bool cond, const char* msg) {
    if (cond) {
        ++g_passed;
        std::printf("[PASS] %s\n", msg);
    } else {
        ++g_failed;
        std::printf("[FAIL] %s\n", msg);
    }
}
} // namespace

int main() {
    // 1. Standard Modbus-RTU CRC-16 Golden Vector (01 03 00 00 00 0A -> CRC 0xCDC5, wire C5 CD)
    const auto req = iiot::ModbusRtuCodec::build_read_request(
        0x01, iiot::FunctionCode::ReadHoldingRegisters, 0x0000, 10);
    check(req[6] == 0xC5U && req[7] == 0xCDU,
          "CRC-16/MODBUS matches golden standard vector (01 03 00 00 00 0A -> C5 CD)");

    // 2. Lock-Free SPSC RingBuffer Capacity, Wrap-Around & Full Protection
    iiot::SpscRingBuffer<uint8_t, 8> rb; // Usable slots = 7
    check(rb.empty(), "SPSC RingBuffer initializes empty");
    for (uint8_t i = 0; i < 7; ++i) {
        check(rb.push(static_cast<uint8_t>(0xA0U + i)), "RingBuffer push slot");
    }
    check(!rb.push(0xFFU), "RingBuffer rejects overflow when full (zero corruption)");
    uint8_t val = 0;
    for (uint8_t i = 0; i < 7; ++i) {
        check(rb.pop(val) && (val == static_cast<uint8_t>(0xA0U + i)), "RingBuffer FIFO order preserved");
    }
    check(rb.empty(), "RingBuffer returns to empty after full drain");

    // 3. Valid Read Holding Registers Response + Leading RS485 Bus Noise Recovery
    // Prepend 2 garbage EMI noise bytes (0xFF, 0x00) before valid 3-register response
    std::vector<uint8_t> wire = {
        0xFF, 0x00,             // RS485 transceiver turn-around glitch bytes
        0x01, 0x03, 0x06,       // Slave 1, FC 0x03, 6 payload bytes
        0x08, 0xCA,             // Reg 0: 2250 -> 225.0 V
        0x08, 0x98,             // Reg 1: 2200 -> 220.0 V
        0xFF, 0x4C,             // Reg 2: -180 (int16) -> -18.0 C
        0x00, 0x00              // Placeholder for CRC
    };
    const uint16_t resp_crc = iiot::crc16_modbus(wire.data() + 2, 9);
    wire[11] = static_cast<uint8_t>(resp_crc & 0xFFU);
    wire[12] = static_cast<uint8_t>((resp_crc >> 8U) & 0xFFU);

    std::size_t offset = 0;
    iiot::ModbusResponse parsed{};
    while (offset < wire.size()) {
        std::size_t consumed = 0;
        const auto st = iiot::ModbusRtuCodec::parse_response(
            wire.data() + offset, wire.size() - offset, parsed, consumed);
        offset += consumed;
        if (st == iiot::ParseStatus::ValidFrame) {
            break;
        }
    }
    check(parsed.slave_id == 1 && parsed.register_count == 3 &&
          parsed.registers[0] == 2250 && parsed.registers[1] == 2200,
          "Sliding-window parser recovers from RS485 EMI noise and decodes 3 registers");

    // 4. Modbus Exception Frame Decoding (Illegal Data Address = 0x02)
    uint8_t exc_frame[5] = {0x02, 0x83, 0x02, 0x00, 0x00};
    const uint16_t exc_crc = iiot::crc16_modbus(exc_frame, 3);
    exc_frame[3] = static_cast<uint8_t>(exc_crc & 0xFFU);
    exc_frame[4] = static_cast<uint8_t>((exc_crc >> 8U) & 0xFFU);
    std::size_t consumed = 0;
    const auto exc_st = iiot::ModbusRtuCodec::parse_response(exc_frame, 5, parsed, consumed);
    check(exc_st == iiot::ParseStatus::ExceptionFrame && parsed.is_exception &&
          parsed.exception_code == 0x02,
          "Modbus exception frame (0x83 / Illegal Address) decoded accurately");

    // 5. Industrial 3-Phase Loss & Thermal Excursion Debounce + Zero-Heap JSON
    iiot::IndustrialFaultMonitor monitor({175.0f, 25.0f, 32.0f, -12.0f, 3});
    iiot::CabinetTelemetry bad_sample{221.0f, 219.0f, 95.0f, 18.4f, -8.5f, 3600};

    check(monitor.evaluate(bad_sample) == iiot::kFaultNone, "Debounce sample 1/3 suppresses transient spike");
    check(monitor.evaluate(bad_sample) == iiot::kFaultNone, "Debounce sample 2/3 suppresses transient spike");
    const uint8_t latched = monitor.evaluate(bad_sample);
    check((latched & iiot::kFaultPhaseLoss) &&
          (latched & iiot::kFaultPhaseAsymmetry) &&
          (latched & iiot::kFaultThermalExcursion),
          "Debounce sample 3/3 latches Phase-Loss, Asymmetry, and Cold-Chain Thermal alarms");

    char json_buf[256]{};
    const int n = iiot::IndustrialFaultMonitor::serialize_mqtt_json(
        json_buf, sizeof(json_buf), "COLD_VAULT_01", bad_sample, latched);
    check(n > 0 && std::strstr(json_buf, "\"alarm\":true") != nullptr,
          "Zero-heap stack JSON serializer emits valid MQTT alarm payload");

    std::printf("==================================================\n");
    std::printf("IIoT Edge Gateway Suite: %d passed, %d failed\n", g_passed, g_failed);
    return (g_failed == 0) ? 0 : 1;
}
