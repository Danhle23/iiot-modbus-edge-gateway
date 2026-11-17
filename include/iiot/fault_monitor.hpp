#pragma once
// ============================================================================
// fault_monitor.hpp - Industrial 3-Phase Power & Cold-Storage Fault Detector
// With Debounce Counter, Phase-Loss Asymmetry Detection & Zero-Heap MQTT JSON
// Author: Le Thanh Danh (Danhle23)
// ============================================================================
#include <algorithm>
#include <cstdint>
#include <cstdio>

namespace iiot {

struct CabinetTelemetry {
    float    voltage_a_v{220.0f};
    float    voltage_b_v{220.0f};
    float    voltage_c_v{220.0f};
    float    current_rms_a{0.0f};
    float    chamber_temp_c{-18.0f};
    uint32_t uptime_sec{0};
};

enum FaultFlags : uint8_t {
    kFaultNone            = 0x00,
    kFaultPhaseLoss       = 0x01, // Any phase < 175V
    kFaultPhaseAsymmetry  = 0x02, // Max-Min phase spread > 25V
    kFaultOverCurrent     = 0x04, // Current > threshold
    kFaultThermalExcursion= 0x08  // Cold storage warmer than max_temp_c
};

struct SafetyThresholds {
    float   min_phase_voltage_v{175.0f};
    float   max_phase_imbalance_v{25.0f};
    float   max_current_a{32.0f};
    float   max_chamber_temp_c{-12.0f};
    uint8_t debounce_samples{3};
};

class IndustrialFaultMonitor {
public:
    explicit constexpr IndustrialFaultMonitor(SafetyThresholds cfg = {}) noexcept
        : cfg_(cfg) {}

    [[nodiscard]] uint8_t evaluate(const CabinetTelemetry& sample) noexcept {
        uint8_t instant = kFaultNone;

        const float v_min = std::min({sample.voltage_a_v, sample.voltage_b_v, sample.voltage_c_v});
        const float v_max = std::max({sample.voltage_a_v, sample.voltage_b_v, sample.voltage_c_v});

        if (v_min < cfg_.min_phase_voltage_v) {
            instant |= kFaultPhaseLoss;
        }
        if ((v_max - v_min) > cfg_.max_phase_imbalance_v) {
            instant |= kFaultPhaseAsymmetry;
        }
        if (sample.current_rms_a > cfg_.max_current_a) {
            instant |= kFaultOverCurrent;
        }
        if (sample.chamber_temp_c > cfg_.max_chamber_temp_c) {
            instant |= kFaultThermalExcursion;
        }

        if (instant != kFaultNone) {
            if (consecutive_faults_ < 255U) {
                ++consecutive_faults_;
            }
            if (consecutive_faults_ >= cfg_.debounce_samples) {
                latched_faults_ = instant;
            }
        } else {
            consecutive_faults_ = 0;
            latched_faults_     = kFaultNone;
        }

        return latched_faults_;
    }

    // Format compact MQTT JSON into fixed stack buffer without heap std::string
    static int serialize_mqtt_json(
        char* out_buf,
        std::size_t cap,
        const char* node_id,
        const CabinetTelemetry& t,
        uint8_t fault_mask) noexcept
    {
        return std::snprintf(
            out_buf,
            cap,
            "{\"node\":\"%s\",\"va\":%.1f,\"vb\":%.1f,\"vc\":%.1f,"
            "\"irms\":%.2f,\"temp_c\":%.2f,\"fault_mask\":%u,\"alarm\":%s,\"up\":%u}",
            node_id,
            static_cast<double>(t.voltage_a_v),
            static_cast<double>(t.voltage_b_v),
            static_cast<double>(t.voltage_c_v),
            static_cast<double>(t.current_rms_a),
            static_cast<double>(t.chamber_temp_c),
            static_cast<unsigned>(fault_mask),
            (fault_mask != 0U) ? "true" : "false",
            static_cast<unsigned>(t.uptime_sec));
    }

private:
    SafetyThresholds cfg_;
    uint8_t          consecutive_faults_{0};
    uint8_t          latched_faults_{kFaultNone};
};

} // namespace iiot
