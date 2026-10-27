/* SPDX-License-Identifier: Apache-2.0 */
// 单板实验台架入口：仅 Discover，不 Arm，不启动机器人运行时。
#include <ecrt.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "hardware/ethercat/pdo_config.hpp"
#include "hardware/ethercat/wire.hpp"
namespace {
using namespace dust_control::ethercat;
struct Copy {
    unsigned image, domain, bytes;
};
void Check(bool value, const char* message) {
    if (!value) {
        throw std::runtime_error(message);
    }
}
void Map(ec_slave_config_t* sc, ec_domain_t* domain,
         const ProcessImageLayout& image, bool output,
         std::vector<Copy>& copies) {
    std::vector<std::vector<ec_pdo_entry_info_t>> entries(
        image.mappings.size());
    std::vector<ec_pdo_info_t> pdos;
    for (size_t i = 0; i < image.mappings.size(); ++i) {
        for (const auto& e : image.mappings[i].entries) {
            entries[i].push_back({e.object_index, e.subindex, e.bit_length});
        }
        pdos.push_back({image.mappings[i].index, unsigned(entries[i].size()),
                        entries[i].data()});
    }
    ec_sync_info_t sync[] = {
        {uint8_t(output ? 2 : 3), output ? EC_DIR_OUTPUT : EC_DIR_INPUT,
         unsigned(pdos.size()), pdos.data(),
         output ? EC_WD_ENABLE : EC_WD_DISABLE},
        {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DEFAULT}};
    Check(ecrt_slave_config_pdos(sc, EC_END, sync) == 0, "PDO mapping failed");
    for (const auto& mapping : image.mappings) {
        for (const auto& e : mapping.entries) {
            unsigned bit = 0;
            int offset = ecrt_slave_config_reg_pdo_entry(
                sc, e.object_index, e.subindex, domain, &bit);
            Check(offset >= 0 && !bit, "PDO registration failed/alignment");
            copies.push_back(
                {e.offset, unsigned(offset), unsigned(e.bit_length / 8)});
        }
    }
}
struct Master {
    ec_master_t* ptr{};
    ~Master() {
        if (ptr) {
            ecrt_release_master(ptr);
        }
    }
};
}  // namespace
int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr
            << "Usage: dust_ethercat_discover LAB_CONFIG.yaml\nOnly Discover, "
               "fixed lab identity, no Arm or peripheral commands.\n";
        return 2;
    }
    try {
        const auto config = LoadPdoConfig(argv[1]);
        Check(config.adapters.size() == 1, "Requires exactly one lab slave");
        const auto& c = config.adapters.begin()->second;
        Check(c.vendor_id == 0 && c.product_code == 0x44544331 && !c.alias &&
                  !c.position,
              "Requires lab VID=0 PID=0x44544331 alias=0 position=0");
        Check(!c.can_tx_slots[0] && !c.can_tx_slots[1] && !c.onboard_imu,
              "No CAN TX/IMU allowed");
        for (const auto& uart : c.uart) {
            Check(!uart.tx_bytes, "No UART TX allowed");
        }
        const auto all = BuildPdoLayout(config);
        const auto& layout = all.adapters.begin()->second;
        Master master{ecrt_request_master(config.master_index)};
        Check(master.ptr, "Cannot request IgH master");
        ec_master_info_t info{};
        Check(ecrt_master(master.ptr, &info) == 0 && info.slave_count == 1,
              "Requires exactly one connected slave");
        ec_slave_info_t identity{};
        Check(ecrt_master_get_slave(master.ptr, 0, &identity) == 0 &&
                  identity.vendor_id == 0 &&
                  identity.product_code == 0x44544331 &&
                  identity.revision_number == 0x10000,
              "Live identity mismatch; update gateway firmware first");
        auto* domain = ecrt_master_create_domain(master.ptr);
        Check(domain, "Cannot create domain");
        auto* sc = ecrt_master_slave_config(master.ptr, c.alias, c.position,
                                            c.vendor_id, c.product_code);
        Check(sc, "Cannot configure slave");
        const uint32_t values[] = {layout.layout_signature,
                                   1000,
                                   20000,
                                   c.can_tx_slots[0],
                                   c.can_tx_slots[1],
                                   c.can_rx_slots[0],
                                   c.can_rx_slots[1],
                                   c.can_rx_slots[2],
                                   c.can_rx_slots[3],
                                   c.uart[0].tx_bytes,
                                   c.uart[0].rx_bytes,
                                   c.uart[1].tx_bytes,
                                   c.uart[1].rx_bytes,
                                   c.uart[2].tx_bytes,
                                   c.uart[2].rx_bytes,
                                   c.uart[3].tx_bytes,
                                   c.uart[3].rx_bytes,
                                   c.uart[4].tx_bytes,
                                   c.uart[4].rx_bytes,
                                   c.uart[5].tx_bytes,
                                   c.uart[5].rx_bytes,
                                   0};
        for (unsigned i = 0; i < 22; ++i) {
            Check(ecrt_slave_config_sdo32(sc, 0x2100, i + 1, values[i]) == 0,
                  "Cannot queue configuration SDO");
        }
        Check(ecrt_slave_config_sdo16(sc, 0x1c32, 1, 0) == 0 &&
                  ecrt_slave_config_sdo16(sc, 0x1c33, 1, 0) == 0,
              "Cannot queue FreeRun synchronization selection");
        std::vector<Copy> output, input;
        Map(sc, domain, layout.rx, true, output);
        Map(sc, domain, layout.tx, false, input);
        Check(ecrt_master_activate(master.ptr) == 0, "Cannot activate master");
        auto* data = ecrt_domain_data(domain);
        Check(data, "Cannot access domain");
        std::vector<uint8_t> rx(layout.rx.bytes), tx(layout.tx.bytes);
        wire::Put32(rx.data(), wire::kMagic);
        wire::Put16(rx.data() + 4, wire::kMajor);
        const uint64_t session =
            uint64_t(
                std::chrono::steady_clock::now().time_since_epoch().count()) |
            1;
        wire::Put64(rx.data() + 8, session);
        wire::Put32(rx.data() + 28, wire::kDiscover);
        wire::Put32(rx.data() + 32, layout.layout_signature);
        wire::Put32(rx.data() + 40, 1000);
        uint64_t boot = 0;
        uint32_t last_sequence = 0;
        unsigned valid = 0, complete = 0, misses = 0;
        auto next = std::chrono::steady_clock::now();
        const auto deadline = next + std::chrono::seconds(15);
        std::cout << "Discover-only: Rx=" << rx.size() << " Tx=" << tx.size()
                  << " signature=" << layout.layout_signature << std::endl;
        for (uint32_t exchange = 1; std::chrono::steady_clock::now() < deadline;
             ++exchange) {
            next += std::chrono::microseconds(1000);
            ecrt_master_receive(master.ptr);
            ecrt_domain_process(domain);
            ec_domain_state_t state{};
            ec_slave_config_state_t slave{};
            ecrt_domain_state(domain, &state);
            ecrt_slave_config_state(sc, &slave);
            if (slave.operational && state.wc_state == EC_WC_COMPLETE) {
                ++complete;
                for (const auto& e : input) {
                    std::memcpy(tx.data() + e.image, data + e.domain, e.bytes);
                }
                if (wire::U32(tx.data()) == wire::kMagic) {
                    Check(wire::U16(tx.data() + 4) == wire::kMajor &&
                              wire::U32(tx.data() + 32) ==
                                  layout.layout_signature,
                          "DTC1 ABI/layout mismatch");
                    Check(wire::U32(tx.data() + 40) == wire::kReady &&
                              !wire::U32(tx.data() + 44),
                          "Slave not Ready or protected/faulted");
                    Check(wire::U32(tx.data() + 36) & wire::kFirmwareProtection,
                          "Firmware IO protection is not installed");
                    const auto current_boot = wire::U64(tx.data() + 8);
                    Check(current_boot && (!boot || boot == current_boot),
                          "Boot identity changed/zero");
                    boot = current_boot;
                    if (wire::U32(tx.data() + 24) != last_sequence &&
                        wire::U64(tx.data() + 16) == session) {
                        const auto sequence = wire::U32(tx.data() + 24);
                        Check(!last_sequence ||
                                  wire::Newer(sequence, last_sequence),
                              "Stale/reordered slave publication");
                        ++valid;
                        last_sequence = sequence;
                    }
                }
            } else if (valid) {
                ++misses;
            }
            wire::Put32(rx.data() + 24, exchange);
            for (const auto& e : output) {
                std::memcpy(data + e.domain, rx.data() + e.image, e.bytes);
            }
            ecrt_domain_queue(domain);
            ecrt_master_send(master.ptr);
            if (exchange % 1000 == 0) {
                std::cout << "AL=" << unsigned(slave.al_state)
                          << " WKC=" << state.working_counter
                          << " complete=" << complete << " fresh=" << valid
                          << " misses=" << misses << " boot=" << boot
                          << std::endl;
            }
            if (valid >= 5000) {
                Check(!misses, "WKC/OP lost after handshake");
                std::cout << "PASS: 5000 fresh Ready replies, no Arm/output "
                             "commands.\n";
                return 0;
            }
            std::this_thread::sleep_until(next);
            if (std::chrono::steady_clock::now() >
                next + std::chrono::milliseconds(5)) {
                next = std::chrono::steady_clock::now();
            }
        }
        throw std::runtime_error(
            "Timed out waiting for 5000 fresh Discover replies");
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
