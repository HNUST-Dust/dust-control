/* SPDX-License-Identifier: Apache-2.0 */
#include "pdo_config.hpp"

#include <algorithm>
#include <cctype>
#include <set>
#include <stdexcept>

namespace dust_control::ethercat {
namespace {
void Require(bool condition, const std::string& error) {
    if (!condition) {
        throw std::invalid_argument(error);
    }
}
template <size_t N>
ProcessImageLayout Image(uint16_t mapping, uint32_t bytes,
                         const std::array<pdo::Entry, N>& entries,
                         const char* name) {
    ProcessImageLayout image{bytes, {{mapping, {}}}};
    for (const auto& entry : entries) {
        image.mappings[0].entries.push_back(
            {name, entry.object, entry.subindex, entry.bits, entry.offset});
    }
    return image;
}
}  // namespace

PdoLayout BuildPdoLayout(const PdoConfig& config) {
    Require(config.cycle_time_us == pdo::kCycleTimeUs,
            "DTG1 requires cycle_time_us=125");
    Require(config.master_index <= 255, "Master index out of range");
    Require(!config.adapters.empty() && config.adapters.size() <= 3,
            "Expected 1..3 adapters");
    Require(config.max_rxpdo_bytes >= pdo::kRxBytes &&
                config.max_rxpdo_bytes <= 4096 &&
                config.max_txpdo_bytes >= pdo::kTxBytes &&
                config.max_txpdo_bytes <= 4096,
            "Invalid process image planning limits");
    Require(config.max_wire_time_us > 0 && config.max_wire_time_us <= 1000,
            "Invalid wire time planning limit");
    PdoLayout result;
    result.master_index = config.master_index;
    result.cycle_time_us = config.cycle_time_us;
    std::set<std::pair<uint16_t, uint16_t>> addresses;
    for (const auto& [name, adapter] : config.adapters) {
        Require(!name.empty() && name.size() <= 8 &&
                    std::all_of(name.begin(), name.end(),
                                [](unsigned char c) {
                                    return std::isalnum(c) || c == '_';
                                }),
                "Adapter names require 1..8 letters, digits or underscores");
        Require(addresses.emplace(adapter.alias, adapter.position).second,
                "Duplicate EtherCAT alias/position");
        AdapterPdoLayout layout;
        layout.alias = adapter.alias;
        layout.position = adapter.position;
        layout.cycle_time_us = config.cycle_time_us;
        layout.rx =
            Image(pdo::kRxMapping, pdo::kRxBytes, pdo::kRxEntries, "device.rx");
        layout.tx =
            Image(pdo::kTxMapping, pdo::kTxBytes, pdo::kTxEntries, "device.tx");
        layout.layout_signature = pdo::kSignature;
        result.total_rx_bytes += layout.rx.bytes;
        result.total_tx_bytes += layout.tx.bytes;
        result.estimated_frames += 2;
        result.adapters.emplace(name, std::move(layout));
    }
    // 100 Mbps 的规划估算，不能替代实测周期验收。
    result.estimated_wire_time_ns =
        (result.total_rx_bytes + result.total_tx_bytes +
         result.estimated_frames * 52) *
        80;
    Require(result.estimated_wire_time_ns <= config.max_wire_time_us * 1000,
            "Estimated aggregate wire time exceeds planning limit");
    Require(result.estimated_wire_time_ns <= config.cycle_time_us * 1000,
            "Estimated aggregate wire time exceeds exchange cycle");
    return result;
}
}  // namespace dust_control::ethercat
