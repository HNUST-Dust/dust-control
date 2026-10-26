/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <atomic>
#include <span>
#include <string>

#include "concurrency/priority_mutex.hpp"
#include "hardware/can_frame.hpp"
#include "pdo_config.hpp"

namespace dust_control::host {
inline constexpr uint32_t kCommandTimeoutUs = 20000;
enum class CanProducer : uint8_t { kChassis, kGimbal };
struct CanCommandGroup {
    uint8_t bus = 0;
    uint16_t id = 0;
    bool operator==(const CanCommandGroup&) const = default;
};
// Business threads only submit. The one EtherCAT master thread consumes.
class EthercatTransmitter {
   public:
    explicit EthercatTransmitter(const ethercat::AdapterPdoConfig& capacity);
    bool ConfigureCan(CanProducer producer,
                      std::span<const CanCommandGroup> groups,
                      uint64_t timeout_ns);
    bool HasCanProducers() const;
    bool SubmitCan(CanProducer producer,
                   std::span<const devices::CanFrame> frames, uint64_t sequence,
                   uint64_t generated_ns);
    bool SubmitUart(uint8_t channel, std::span<const uint8_t> bytes,
                    uint64_t generated_ns, uint64_t lifetime_ns = 200000000);
    bool healthy() const {
        return active_.load() && !stopped_.load();
    }
    std::string error() const;
    void RequestStop();

    // Called by the master, except registration before startup.
    void Activate(uint64_t now_ns);
    void SealRegistration();
    void Fail(const std::string& reason);
    uint32_t source_mask() const;
    uint32_t can_channels() const;
    struct Batch {
        std::array<devices::CanFrame, 16> frames{};
        size_t count = 0;
        uint32_t sequence = 0;
        uint32_t remaining_us = 0;
        bool valid = false;
    };
    bool ReadBatch(unsigned source, uint64_t now_ns, Batch& batch);
    struct UartBlock {
        std::array<uint8_t, 512> bytes{};
        uint16_t size = 0;
        uint32_t sequence = 0;
        bool valid = false;
    };
    bool NextUart(unsigned port, uint32_t ack, bool ack_valid, uint64_t now_ns,
                  UartBlock& block);

   private:
    struct Source {
        std::array<CanCommandGroup, 16> groups{};
        size_t group_count = 0;
        uint64_t timeout_ns = 0;
        uint64_t generated_ns = 0;
        uint64_t sequence = 0;
        Batch batch;
    };
    struct Message {
        std::array<uint8_t, 512> bytes{};
        uint16_t size = 0, cursor = 0;
        uint64_t generated_ns = 0, lifetime_ns = 0;
    };
    struct Pending {
        UartBlock block;
        uint64_t generated_ns = 0, lifetime_ns = 0;
        uint64_t first_sent_ns = 0;
        bool sent = false;
    };
    struct Uart {
        std::array<Message, 16> queue{};
        size_t head = 0, count = 0;
        std::array<Pending, 4> window{};
        size_t window_head = 0, window_count = 0, cursor = 0;
        uint32_t next_sequence = 1, last_ack = 0;
        bool have_ack = false;
    };
    const ethercat::AdapterPdoConfig capacity_;
    mutable concurrency::PriorityMutex mutex_;
    std::array<Source, 2> sources_{};
    std::array<Uart, 6> uart_{};
    uint64_t started_ns_ = 0;
    bool started_ = false, registration_closed_ = false;
    std::atomic<bool> active_{false}, stopped_{false};
    std::string error_;
};
class CanCommandPort {
   public:
    CanCommandPort(EthercatTransmitter& transmitter, CanProducer producer)
        : transmitter_(transmitter), producer_(producer) {}
    bool Configure(std::span<const CanCommandGroup> groups,
                   uint64_t timeout_ns) const {
        return transmitter_.ConfigureCan(producer_, groups, timeout_ns);
    }
    bool Submit(std::span<const devices::CanFrame> frames, uint64_t sequence,
                uint64_t generated_ns) const {
        return transmitter_.SubmitCan(producer_, frames, sequence,
                                      generated_ns);
    }

   private:
    EthercatTransmitter& transmitter_;
    CanProducer producer_;
};
class UartFramePort {
   public:
    explicit UartFramePort(EthercatTransmitter& transmitter)
        : transmitter_(transmitter) {}
    bool Submit(uint8_t channel, std::span<const uint8_t> bytes,
                uint64_t generated_ns, uint64_t lifetime_ns = 200000000) const {
        return transmitter_.SubmitUart(channel, bytes, generated_ns,
                                       lifetime_ns);
    }

   private:
    EthercatTransmitter& transmitter_;
};
}  // namespace dust_control::host
