/* SPDX-License-Identifier: Apache-2.0 */
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

#include "pdo_config.hpp"

namespace {
using dust_control::ethercat::ProcessImageLayout;
void PrintImage(const ProcessImageLayout& image) {
    std::cout << "{\"bytes\":" << image.bytes << ",\"mappings\":[";
    bool first_mapping = true;
    for (const auto& mapping : image.mappings) {
        if (!first_mapping) {
            std::cout << ',';
        }
        first_mapping = false;
        std::cout << "{\"index\":" << mapping.index << ",\"entries\":[";
        bool first_entry = true;
        for (const auto& entry : mapping.entries) {
            if (!first_entry) {
                std::cout << ',';
            }
            first_entry = false;
            std::cout << "{\"name\":" << std::quoted(entry.name)
                      << ",\"object_index\":" << entry.object_index
                      << ",\"subindex\":" << unsigned(entry.subindex)
                      << ",\"bit_length\":" << unsigned(entry.bit_length)
                      << ",\"offset\":" << entry.offset << '}';
        }
        std::cout << "]}";
    }
    std::cout << "]}";
}
}  // namespace
int main(int argc, char** argv) {
    const bool json = argc == 3 && std::string(argv[2]) == "--json";
    if (argc < 2 || argc > 3 || (argc == 3 && !json)) {
        std::cerr
            << "Usage: dust_ethercat_layout CONFIG.yaml [--json]\n"
               "Offline candidate layout only; no hardware configuration.\n";
        return 2;
    }
    try {
        const auto layout = dust_control::ethercat::BuildPdoLayout(
            dust_control::ethercat::LoadPdoConfig(argv[1]));
        if (json) {
            std::cout << "{\"layout_version\":" << layout.version
                      << ",\"runtime_connected\":false,\"master_index\":"
                      << layout.master_index
                      << ",\"cycle_time_us\":" << layout.cycle_time_us
                      << ",\"total_rx_bytes\":" << layout.total_rx_bytes
                      << ",\"total_tx_bytes\":" << layout.total_tx_bytes
                      << ",\"estimated_frames\":" << layout.estimated_frames
                      << ",\"estimated_wire_time_ns\":"
                      << layout.estimated_wire_time_ns << ",\"adapters\":{";
            bool first = true;
            for (const auto& [name, adapter] : layout.adapters) {
                if (!first) {
                    std::cout << ',';
                }
                first = false;
                std::cout << std::quoted(name)
                          << ":{\"alias\":" << adapter.alias
                          << ",\"position\":" << adapter.position
                          << ",\"layout_signature\":"
                          << adapter.layout_signature << ",\"rx\":";
                PrintImage(adapter.rx);
                std::cout << ",\"tx\":";
                PrintImage(adapter.tx);
                std::cout << '}';
            }
            std::cout << "}}\n";
        } else {
            std::cout
                << "Offline candidate PDO layout (runtime not connected)\n"
                << "master=" << layout.master_index
                << " cycle=" << layout.cycle_time_us << " us\n";
            for (const auto& [name, adapter] : layout.adapters) {
                std::cout << name << ": alias=" << adapter.alias
                          << " position=" << adapter.position
                          << " RxPDO=" << adapter.rx.bytes
                          << " B TxPDO=" << adapter.tx.bytes
                          << " B signature=0x" << std::hex
                          << adapter.layout_signature << std::dec << '\n';
            }
            std::cout << "Total: RxPDO=" << layout.total_rx_bytes
                      << " B TxPDO=" << layout.total_tx_bytes
                      << " B estimated_frames=" << layout.estimated_frames
                      << " estimated_wire_time="
                      << layout.estimated_wire_time_ns / 1000.0
                      << " us (planning estimate, not measured)\n";
        }
    } catch (const std::exception& error) {
        std::cerr << "EtherCAT PDO configuration: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
