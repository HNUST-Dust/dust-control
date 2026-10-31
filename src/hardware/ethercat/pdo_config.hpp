/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "pdo_schema.hpp"

namespace dust_control::ethercat {
// Offline layout schema; this does not authorize hardware outputs.
inline constexpr uint16_t kLayoutVersion = 1;
struct UartCapacity {
    uint16_t tx_bytes = 0;  // Host -> slave.
    uint16_t rx_bytes = 0;  // Slave -> host.
};
struct AdapterPdoConfig {
    uint16_t alias = 0;
    uint16_t position = 0;
    uint32_t vendor_id = 0;
    uint32_t product_code = 0;
    std::array<uint8_t, 2> can_tx_slots{};  // chassis, gimbal
    std::array<uint8_t, 4> can_rx_slots{};
    std::array<UartCapacity, 6> uart{};
    bool onboard_imu = true;
};
struct PdoConfig {
    uint32_t master_index = 0;
    uint32_t cycle_time_us = 1000;
    // Planning limits, not claimed ESC hardware capabilities.
    uint32_t max_rxpdo_bytes = 4096;
    uint32_t max_txpdo_bytes = 4096;
    uint32_t max_wire_time_us = 800;
    std::map<std::string, AdapterPdoConfig> adapters;
};
enum class PdoDirection { kRx, kTx };  // Slave perspective.
struct PdoEntry {
    std::string name;
    uint16_t object_index = 0;
    uint8_t subindex = 0;
    uint8_t bit_length = 0;
    uint32_t offset = 0;
};
struct PdoMapping {
    uint16_t index = 0;
    std::vector<PdoEntry> entries;
};
struct ProcessImageLayout {
    uint32_t bytes = 0;
    std::vector<PdoMapping> mappings;
};
struct AdapterPdoLayout {
    uint16_t alias = 0;
    uint16_t position = 0;
    ProcessImageLayout rx;
    ProcessImageLayout tx;
    // CRC32 of version + ordered mappings, field names, offsets and lengths.
    // A layout compatibility check, not an authentication primitive.
    uint32_t layout_signature = 0;
    uint32_t cycle_time_us = 1000;
};
struct PdoLayout {
    uint16_t version = kLayoutVersion;
    uint32_t master_index = 0;
    uint32_t cycle_time_us = 1000;
    uint32_t total_rx_bytes = 0;
    uint32_t total_tx_bytes = 0;
    uint32_t estimated_frames = 0;
    uint32_t estimated_wire_time_ns = 0;
    std::map<std::string, AdapterPdoLayout> adapters;
};
// PDO capacity from the schema's fixed production profile (single source of
// truth). Identity/master fields are filled by the caller from application
// configuration, not duplicated here.
inline AdapterPdoConfig ProductionCapacity() {
    const auto p = dust::pdo::ProductionConfig();
    AdapterPdoConfig adapter;
    adapter.can_tx_slots = p.can_tx;
    adapter.can_rx_slots = p.can_rx;
    for (unsigned i = 0; i < 6; ++i) {
        adapter.uart[i] = {p.uart_tx[i], p.uart_rx[i]};
    }
    adapter.onboard_imu = p.imu;
    return adapter;
}
// Complete production PdoConfig for offline preview: schema capacity plus the
// production defaults (master 0, placeholder identity). The runtime overrides
// identity/master from application configuration.
inline PdoConfig ProductionPdoConfig() {
    PdoConfig config;
    config.master_index = 0;
    config.cycle_time_us = dust::pdo::kProductionCycleUs;
    config.max_wire_time_us = dust::pdo::kProductionCycleUs;
    config.adapters["base"] = ProductionCapacity();
    return config;
}
PdoConfig LoadPdoConfig(const std::string& file);
PdoLayout BuildPdoLayout(const PdoConfig& config);
}  // namespace dust_control::ethercat
