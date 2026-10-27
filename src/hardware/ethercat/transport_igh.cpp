/* SPDX-License-Identifier: Apache-2.0 */
#include <algorithm>
#include <cstring>
#include <stdexcept>

#include "transport.hpp"
#if DUST_HAVE_IGH
#include <ecrt.h>
#endif
namespace dust_control::ethercat {
#if DUST_HAVE_IGH
namespace {
struct CopyEntry {
    uint32_t image_offset;
    unsigned domain_offset, size;
};
struct Slave {
    ec_slave_config_t* config = nullptr;
    std::vector<CopyEntry> rx, tx;
};
class IghTransport final : public Transport {
   public:
    ~IghTransport() override {
        Close();
    }
    bool Open(uint32_t index, std::vector<BoardImage>& boards,
              std::string& error) override {
        try {
            for (const auto& board : boards) {
                Check(board.config.vendor_id && board.config.product_code,
                      "Set nonzero vendor_id/product_code from the actual "
                      "slave identity");
            }
            master_ = ecrt_request_master(index);
            Check(
                master_,
                "Cannot request IgH master; check service/device permissions");
            domain_ = ecrt_master_create_domain(master_);
            Check(domain_, "Cannot create IgH domain");
            slaves_.resize(boards.size());
            for (size_t i = 0; i < boards.size(); ++i) {
                const auto& board = boards[i];
                const auto& c = board.config;
                Check(c.vendor_id && c.product_code,
                      "Set nonzero vendor_id/product_code from the actual "
                      "slave identity");
                auto& slave = slaves_[i];
                slave.config = ecrt_master_slave_config(
                    master_, c.alias, c.position, c.vendor_id, c.product_code);
                Check(slave.config, "EtherCAT slave configuration failed");
                // Vendor object 0x2100 is the startup layout contract. It must
                // be implemented by the slave before enabling this host
                // runtime.
                Check(
                    ecrt_slave_config_sdo32(slave.config, 0x2100, 1,
                                            board.layout.layout_signature) == 0,
                    "Cannot queue slave layout signature SDO");
                Check(
                    ecrt_slave_config_sdo32(slave.config, 0x2100, 2, 1000) == 0,
                    "Cannot queue slave cycle SDO");
                Check(ecrt_slave_config_sdo32(slave.config, 0x2100, 3, 20000) ==
                          0,
                      "Cannot queue slave command watchdog SDO");
                // Capacity descriptor, split across distinct subindices.
                unsigned sub = 4;
                for (auto n : c.can_tx_slots) {
                    Sdo(slave.config, sub++, n);
                }
                for (auto n : c.can_rx_slots) {
                    Sdo(slave.config, sub++, n);
                }
                for (auto n : c.uart) {
                    Sdo(slave.config, sub++, n.tx_bytes);
                    Sdo(slave.config, sub++, n.rx_bytes);
                }
                Sdo(slave.config, sub, c.onboard_imu ? 1 : 0);
                // SSC 未显式收到同步方式时会自动选 SM Sync；首版网关仅支持
                // FreeRun。
                Check(
                    ecrt_slave_config_sdo16(slave.config, 0x1c32, 1, 0) == 0 &&
                        ecrt_slave_config_sdo16(slave.config, 0x1c33, 1, 0) ==
                            0,
                    "Cannot queue FreeRun synchronization selection SDO");
                Map(slave, board.layout.rx, 2, EC_DIR_OUTPUT, EC_WD_ENABLE,
                    slave.rx);
                Map(slave, board.layout.tx, 3, EC_DIR_INPUT, EC_WD_DISABLE,
                    slave.tx);
            }
            Check(ecrt_master_activate(master_) == 0,
                  "IgH master activation failed");
            data_ = ecrt_domain_data(domain_);
            Check(data_, "IgH domain data unavailable");
            return true;
        } catch (const std::exception& e) {
            error = e.what();
            Close();
            return false;
        }
    }
    bool Receive(std::vector<BoardImage>& boards, std::string&) override {
        ecrt_master_receive(master_);
        ecrt_domain_process(domain_);
        ec_domain_state_t domain{};
        ecrt_domain_state(domain_, &domain);
        ec_master_state_t master{};
        ecrt_master_state(master_, &master);
        for (size_t i = 0; i < boards.size(); ++i) {
            ec_slave_config_state_t state{};
            ecrt_slave_config_state(slaves_[i].config, &state);
            boards[i].operational = master.link_up && state.operational;
            boards[i].complete = domain.wc_state == EC_WC_COMPLETE;
            if (boards[i].complete && boards[i].operational) {
                for (const auto& e : slaves_[i].tx) {
                    std::memcpy(boards[i].input.data() + e.image_offset,
                                data_ + e.domain_offset, e.size);
                }
            }
        }
        return true;
    }
    bool Send(const std::vector<BoardImage>& boards, std::string&) override {
        for (size_t i = 0; i < boards.size(); ++i) {
            for (const auto& e : slaves_[i].rx) {
                std::memcpy(data_ + e.domain_offset,
                            boards[i].output.data() + e.image_offset, e.size);
            }
        }
        ecrt_domain_queue(domain_);
        ecrt_master_send(master_);
        return true;
    }
    void Close() override {
        if (master_) {
            ecrt_release_master(master_);
        }
        master_ = nullptr;
        domain_ = nullptr;
        data_ = nullptr;
        slaves_.clear();
    }

   private:
    static void Check(bool good, const char* reason) {
        if (!good) {
            throw std::runtime_error(reason);
        }
    }
    static void Sdo(ec_slave_config_t* sc, unsigned sub, uint32_t value) {
        Check(ecrt_slave_config_sdo32(sc, 0x2100, sub, value) == 0,
              "Cannot queue slave capacity SDO");
    }
    void Map(Slave& slave, const ProcessImageLayout& image, uint8_t sm,
             ec_direction_t direction, ec_watchdog_mode_t watchdog,
             std::vector<CopyEntry>& copies) {
        std::vector<std::vector<ec_pdo_entry_info_t>> entries(
            image.mappings.size());
        std::vector<ec_pdo_info_t> pdos;
        pdos.reserve(image.mappings.size());
        for (size_t i = 0; i < image.mappings.size(); ++i) {
            for (const auto& e : image.mappings[i].entries) {
                entries[i].push_back(
                    {e.object_index, e.subindex, e.bit_length});
            }
            pdos.push_back({image.mappings[i].index,
                            static_cast<unsigned>(entries[i].size()),
                            entries[i].data()});
        }
        ec_sync_info_t sync[] = {
            {sm, direction, static_cast<unsigned>(pdos.size()), pdos.data(),
             watchdog},
            {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DEFAULT}};
        Check(ecrt_slave_config_pdos(slave.config, EC_END, sync) == 0,
              "IgH dynamic PDO mapping failed");
        for (const auto& mapping : image.mappings) {
            for (const auto& entry : mapping.entries) {
                unsigned bit = 0;
                const auto offset = ecrt_slave_config_reg_pdo_entry(
                    slave.config, entry.object_index, entry.subindex, domain_,
                    &bit);
                Check(offset >= 0 && bit == 0,
                      "PDO entry registration failed or is not byte aligned");
                copies.push_back({entry.offset, static_cast<unsigned>(offset),
                                  unsigned(entry.bit_length / 8)});
            }
        }
    }
    ec_master_t* master_ = nullptr;
    ec_domain_t* domain_ = nullptr;
    uint8_t* data_ = nullptr;
    std::vector<Slave> slaves_;
};
}  // namespace
std::unique_ptr<Transport> CreateIghTransport() {
    return std::make_unique<IghTransport>();
}
#else
namespace {
class UnavailableTransport final : public Transport {
   public:
    bool Open(uint32_t, std::vector<BoardImage>&, std::string& error) override {
        error =
            "IgH support is not built; configure DUST_BUILD_IGH=ON with "
            "ecrt.h/libethercat installed";
        return false;
    }
    bool Receive(std::vector<BoardImage>&, std::string&) override {
        return false;
    }
    bool Send(const std::vector<BoardImage>&, std::string&) override {
        return false;
    }
    void Close() override {}
};
}  // namespace
std::unique_ptr<Transport> CreateIghTransport() {
    return std::make_unique<UnavailableTransport>();
}
#endif
}  // namespace dust_control::ethercat
