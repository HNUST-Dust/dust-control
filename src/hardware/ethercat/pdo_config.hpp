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
inline constexpr uint16_t kLayoutVersion = pdo::kLayoutVersion;
struct AdapterPdoConfig {
    uint16_t alias = 0;
    uint16_t position = 0;
    uint32_t vendor_id = 0;
    uint32_t product_code = 0;
};
struct PdoConfig {
    uint32_t master_index = 0;
    uint32_t cycle_time_us = pdo::kCycleTimeUs;
    // Planning limits, not claimed ESC hardware capabilities.
    uint32_t max_rxpdo_bytes = 4096;
    uint32_t max_txpdo_bytes = 4096;
    uint32_t max_wire_time_us = pdo::kCycleTimeUs;
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
    // 固件协议签名；修改布局必须同步从站固件。
    uint32_t layout_signature = 0;
    uint32_t cycle_time_us = pdo::kCycleTimeUs;
};
struct PdoLayout {
    uint16_t version = kLayoutVersion;
    uint32_t master_index = 0;
    uint32_t cycle_time_us = pdo::kCycleTimeUs;
    uint32_t total_rx_bytes = 0;
    uint32_t total_tx_bytes = 0;
    uint32_t estimated_frames = 0;
    uint32_t estimated_wire_time_ns = 0;
    std::map<std::string, AdapterPdoLayout> adapters;
};
PdoLayout BuildPdoLayout(const PdoConfig& config);
}  // namespace dust_control::ethercat
