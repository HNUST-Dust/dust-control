/* SPDX-License-Identifier: Apache-2.0 */
#include "pdo_config.hpp"

#include <algorithm>
#include <cctype>
#include <set>
#include <stdexcept>
#include <utility>

#include "yaml_validation.hpp"

namespace dust_control::ethercat {
namespace {
namespace validation = yaml_validation;
uint32_t Number(const YAML::Node& node, uint32_t maximum) {
    return static_cast<uint32_t>(
        validation::ParseUnsignedInteger(node, maximum));
}
void Require(bool condition, const std::string& error) {
    if (!condition) {
        throw std::invalid_argument(error);
    }
}
void Mapping(const YAML::Node& node, const char* name) {
    Require(node.IsMap(), std::string(name) + " must be a mapping");
}
class ImageBuilder {
   public:
    explicit ImageBuilder(uint16_t first_mapping)
        : next_mapping_(first_mapping) {}
    void Object(uint16_t index, std::string name) {
        object_ = index;
        subindex_ = 0;
        prefix_ = std::move(name);
    }
    void Field(const std::string& name, uint8_t bytes) {
        Require(bytes && bytes <= 30, "Invalid PDO field length");
        Require(subindex_ < 255, "Too many subindices in PDO object");
        if (image_.mappings.empty() ||
            image_.mappings.back().entries.size() == 255) {
            Require(image_.mappings.size() < 255, "Too many PDO mappings");
            image_.mappings.push_back({next_mapping_++, {}});
        }
        image_.mappings.back().entries.push_back(
            {prefix_ + "." + name, object_, static_cast<uint8_t>(++subindex_),
             static_cast<uint8_t>(bytes * 8), image_.bytes});
        image_.bytes += bytes;
    }
    void Bytes(const std::string& name, uint32_t bytes) {
        // ec_pdo_entry_info_t has an 8-bit bit_length. Byte arrays are
        // represented as several <=240-bit entries with a stable chunk rule.
        for (uint32_t offset = 0; offset < bytes; offset += 30) {
            Field(name + "[" + std::to_string(offset) + "]",
                  static_cast<uint8_t>(std::min(30U, bytes - offset)));
        }
    }
    ProcessImageLayout Finish() {
        return std::move(image_);
    }

   private:
    uint16_t next_mapping_;
    uint16_t object_ = 0;
    unsigned subindex_ = 0;
    std::string prefix_;
    ProcessImageLayout image_;
};
void RxHeader(ImageBuilder& b) {
    b.Object(0x2000, "header");
    b.Field("magic", 4);
    b.Field("abi_major", 2);
    b.Field("abi_minor", 2);
    b.Field("session_id", 8);
    b.Field("expected_boot_id", 8);
    b.Field("exchange_sequence", 4);
    b.Field("control_flags", 4);
    b.Field("configuration_signature", 4);
    b.Field("active_source_mask", 4);
    b.Field("cycle_time_us", 4);
    b.Field("status_request_sequence", 4);
    b.Field("status_target", 1);
    b.Field("status_channel", 1);
    b.Bytes("reserved", 14);
}
void TxHeader(ImageBuilder& b) {
    b.Object(0x3000, "header");
    b.Field("magic", 4);
    b.Field("abi_major", 2);
    b.Field("abi_minor", 2);
    b.Field("boot_id", 8);
    b.Field("accepted_session_id", 8);
    for (const char* name :
         {"publish_sequence", "slave_now_us", "configuration_signature",
          "capability_mask", "business_state", "fault_flags",
          "accepted_chassis_sequence", "accepted_gimbal_sequence",
          "status_reply_sequence", "reserved"}) {
        b.Field(name, 4);
    }
}
void CanSource(ImageBuilder& b, unsigned source, unsigned slots) {
    if (!slots) {
        return;
    }
    b.Object(static_cast<uint16_t>(0x2010 + source),
             source ? "can.gimbal" : "can.chassis");
    b.Field("source_sequence", 4);
    b.Field("remaining_valid_us", 4);
    b.Field("flags", 2);
    b.Field("frame_count", 1);
    b.Field("source_id", 1);
    b.Field("reserved", 4);
    for (unsigned i = 0; i < slots; ++i) {
        const auto frame = "frames[" + std::to_string(i) + "].";
        b.Field(frame + "channel", 1);
        b.Field(frame + "length", 1);
        b.Field(frame + "flags", 1);
        b.Field(frame + "reserved", 1);
        b.Field(frame + "id", 4);
        b.Bytes(frame + "data", 8);
    }
}
void CanFeedback(ImageBuilder& b, unsigned bus, unsigned slots) {
    if (!slots) {
        return;
    }
    b.Object(static_cast<uint16_t>(0x3010 + bus), "can" + std::to_string(bus));
    b.Field("batch_sequence", 4);
    b.Field("cumulative_drops", 4);
    b.Field("frame_count", 2);
    b.Field("flags", 2);
    b.Field("reserved", 4);
    for (unsigned i = 0; i < slots; ++i) {
        const auto frame = "frames[" + std::to_string(i) + "].";
        b.Field(frame + "id", 4);
        b.Field(frame + "bus_sequence", 4);
        b.Field(frame + "capture_time_us", 4);
        b.Field(frame + "length", 1);
        b.Field(frame + "flags", 1);
        b.Field(frame + "reserved", 2);
        b.Bytes(frame + "data", 8);
    }
}
void Uart(ImageBuilder& b, unsigned port, unsigned bytes, bool tx_image) {
    b.Object(static_cast<uint16_t>((tx_image ? 0x3020 : 0x2020) + port),
             "uart" + std::to_string(port + 1));
    b.Field("block_sequence", 4);
    b.Field("peer_ack_sequence", 4);
    b.Field("capture_time_us", 4);
    b.Field("length", 2);
    b.Field("flags", 2);
    b.Bytes("data", bytes);
}
void Imu(ImageBuilder& b) {
    b.Object(0x3030, "imu");
    b.Field("sample_sequence", 4);
    b.Field("capture_time_us", 4);
    b.Field("flags", 4);
    b.Field("temperature_c", 4);
    b.Bytes("spi_bytes", 25);
    b.Bytes("reserved", 7);
}
void Diagnostics(ImageBuilder& b) {
    b.Object(0x3040, "diagnostics");
    for (const char* name :
         {"stop_reason", "rejected", "can_drops", "uart_drops",
          "protection_errors", "configuration_errors", "firmware_flags",
          "reserved"}) {
        b.Field(name, 4);
    }
    for (unsigned bus = 0; bus < 4; ++bus) {
        for (const char* name :
             {"received", "submitted", "completed", "dropped", "errors",
              "queue_depth", "controller_state", "packed_error_counts"}) {
            b.Field("can" + std::to_string(bus) + "." + name, 4);
        }
    }
    for (unsigned port = 1; port <= 6; ++port) {
        b.Field("uart" + std::to_string(port) + ".rx_drops", 4);
        b.Field("uart" + std::to_string(port) + ".tx_errors", 4);
    }
    b.Field("status_result", 4);
    b.Field("status_value0", 4);
    b.Field("status_value1", 4);
    b.Field("status_value2", 4);
}
// Portable incremental CRC32. Explicit widths and separators avoid relying
// on host struct layout, std::hash, YAML order or machine endianness.
class Signature {
   public:
    void Byte(uint8_t value) {
        crc_ ^= value;
        for (unsigned i = 0; i < 8; ++i) {
            crc_ = (crc_ >> 1) ^ ((crc_ & 1) ? 0xedb88320U : 0U);
        }
    }
    void Integer(uint32_t value) {
        for (unsigned i = 0; i < 4; ++i) {
            Byte(static_cast<uint8_t>(value >> (i * 8)));
        }
    }
    void Image(const ProcessImageLayout& image) {
        Integer(image.bytes);
        Integer(static_cast<uint32_t>(image.mappings.size()));
        for (const auto& mapping : image.mappings) {
            Integer(mapping.index);
            Integer(static_cast<uint32_t>(mapping.entries.size()));
            for (const auto& entry : mapping.entries) {
                Integer(entry.object_index);
                Integer(entry.subindex);
                Integer(entry.bit_length);
                Integer(entry.offset);
                for (const unsigned char c : entry.name) {
                    Byte(c);
                }
                Byte(0);
            }
        }
    }
    uint32_t Finish() const {
        return crc_ ^ 0xffffffffU;
    }

   private:
    uint32_t crc_ = 0xffffffffU;
};
}  // namespace

PdoConfig LoadPdoConfig(const std::string& file) {
    const auto root = YAML::LoadFile(file);
    validation::ValidateKeys(root, {"schema_version", "ethercat", "adapters"});
    Require(Number(root["schema_version"], 1) == 1,
            "Unsupported PDO configuration schema");
    PdoConfig config;
    const auto master = root["ethercat"];
    validation::ValidateKeys(master, {"master_index", "cycle_time_us"},
                             {"limits"});
    config.master_index = Number(master["master_index"], 255);
    config.cycle_time_us = Number(master["cycle_time_us"], 1000);
    if (const auto limits = master["limits"]) {
        validation::ValidateKeys(
            limits, {"max_rxpdo_bytes", "max_txpdo_bytes", "max_wire_time_us"});
        config.max_rxpdo_bytes = Number(limits["max_rxpdo_bytes"], 4096);
        config.max_txpdo_bytes = Number(limits["max_txpdo_bytes"], 4096);
        config.max_wire_time_us = Number(limits["max_wire_time_us"], 1000);
    }
    const auto adapters = root["adapters"];
    Mapping(adapters, "adapters");
    for (const auto& item : adapters) {
        const auto name = item.first.as<std::string>();
        const auto node = item.second;
        validation::ValidateKeys(node, {"alias", "position", "pdo"});
        AdapterPdoConfig adapter;
        adapter.alias = static_cast<uint16_t>(Number(node["alias"], 65535));
        adapter.position =
            static_cast<uint16_t>(Number(node["position"], 65535));
        const auto pdo = node["pdo"];
        validation::ValidateKeys(pdo, {"can", "uart", "onboard_imu"});
        const auto can = pdo["can"];
        validation::ValidateKeys(can, {"tx_slots", "rx_slots"});
        const auto slots = can["tx_slots"];
        validation::ValidateKeys(slots, {"chassis", "gimbal"});
        adapter.can_tx_slots = {
            static_cast<uint8_t>(Number(slots["chassis"], 16)),
            static_cast<uint8_t>(Number(slots["gimbal"], 16))};
        const auto rx = can["rx_slots"];
        Mapping(rx, "CAN rx_slots");
        std::set<unsigned> buses;
        for (const auto& bus : rx) {
            const auto channel = Number(bus.first, 3);
            Require(buses.insert(channel).second, "Duplicate CAN channel");
            adapter.can_rx_slots[channel] =
                static_cast<uint8_t>(Number(bus.second, 32));
        }
        const auto uart = pdo["uart"];
        Mapping(uart, "UART capacities");
        std::set<unsigned> ports;
        for (const auto& port : uart) {
            const auto channel = Number(port.first, 6);
            Require(channel >= 1 && ports.insert(channel).second,
                    "Invalid or duplicate UART channel");
            validation::ValidateKeys(port.second, {"tx_bytes", "rx_bytes"});
            adapter.uart[channel - 1] = {
                static_cast<uint16_t>(Number(port.second["tx_bytes"], 512)),
                static_cast<uint16_t>(Number(port.second["rx_bytes"], 512))};
        }
        adapter.onboard_imu = pdo["onboard_imu"].as<bool>();
        Require(config.adapters.emplace(name, adapter).second,
                "Duplicate adapter name");
    }
    // Validate numeric and programmatic configuration with one shared path.
    (void)BuildPdoLayout(config);
    return config;
}

PdoLayout BuildPdoLayout(const PdoConfig& config) {
    Require(config.cycle_time_us == 1000,
            "Only cycle_time_us: 1000 is supported");
    Require(config.master_index <= 255, "Master index out of range");
    Require(config.adapters.size() >= 1 && config.adapters.size() <= 3,
            "Expected 1..3 adapters");
    Require(config.max_rxpdo_bytes >= 64 && config.max_rxpdo_bytes <= 4096 &&
                config.max_txpdo_bytes >= 288 && config.max_txpdo_bytes <= 4096,
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
        ImageBuilder rx(0x1600), tx(0x1a00);
        RxHeader(rx);
        TxHeader(tx);
        for (unsigned i = 0; i < 2; ++i) {
            Require(adapter.can_tx_slots[i] <= 16,
                    "CAN source capacity exceeds 16");
            CanSource(rx, i, adapter.can_tx_slots[i]);
        }
        for (unsigned i = 0; i < 4; ++i) {
            Require(adapter.can_rx_slots[i] <= 32,
                    "CAN bus capacity exceeds 32");
            CanFeedback(tx, i, adapter.can_rx_slots[i]);
        }
        for (unsigned i = 0; i < 6; ++i) {
            const auto capacity = adapter.uart[i];
            Require(capacity.tx_bytes <= 512 && capacity.rx_bytes <= 512,
                    "UART capacity exceeds 512");
            // Even a one-way stream needs its ACK in the reverse image.
            // Omit the channel only when both directions are disabled.
            if (capacity.tx_bytes || capacity.rx_bytes) {
                Uart(rx, i, capacity.tx_bytes, false);
                Uart(tx, i, capacity.rx_bytes, true);
            }
        }
        if (adapter.onboard_imu) {
            Imu(tx);
        }
        Diagnostics(tx);
        AdapterPdoLayout layout;
        layout.alias = adapter.alias;
        layout.position = adapter.position;
        layout.rx = rx.Finish();
        layout.tx = tx.Finish();
        Require(layout.rx.bytes <= config.max_rxpdo_bytes &&
                    layout.tx.bytes <= config.max_txpdo_bytes,
                "Adapter " + name + " exceeds process image planning limits");
        Signature signature;
        signature.Integer(kLayoutVersion);
        signature.Integer(config.cycle_time_us);
        signature.Image(layout.rx);
        signature.Image(layout.tx);
        layout.layout_signature = signature.Finish();
        result.total_rx_bytes += layout.rx.bytes;
        result.total_tx_bytes += layout.tx.bytes;
        result.estimated_frames +=
            (layout.rx.bytes + 1399) / 1400 + (layout.tx.bytes + 1399) / 1400;
        result.adapters.emplace(name, std::move(layout));
    }
    // Separate input/output traffic, one datagram per <=1400 B frame,
    // approx. 52 B overhead at 100 Mbps. Not a real-time guarantee.
    result.estimated_wire_time_ns =
        (result.total_rx_bytes + result.total_tx_bytes +
         result.estimated_frames * 52) *
        80;
    Require(result.estimated_wire_time_ns <= config.max_wire_time_us * 1000,
            "Estimated aggregate wire time exceeds planning limit");
    return result;
}
}  // namespace dust_control::ethercat
