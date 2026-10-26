/* SPDX-License-Identifier: Apache-2.0 */
#include "commands.hpp"

#include <algorithm>
#include <mutex>

#include "chassis/runtime/realtime_loop.hpp"

namespace dust_control::host {
EthercatTransmitter::EthercatTransmitter(
    const ethercat::AdapterPdoConfig& capacity)
    : capacity_(capacity) {}
bool EthercatTransmitter::ConfigureCan(CanProducer producer,
                                       std::span<const CanCommandGroup> groups,
                                       uint64_t timeout_ns) {
    const std::lock_guard lock(mutex_);
    const auto index = static_cast<size_t>(producer);
    if (started_ || registration_closed_ || index >= sources_.size() ||
        groups.empty() || groups.size() > capacity_.can_tx_slots[index] ||
        !timeout_ns || timeout_ns > kCommandTimeoutUs * 1000ULL ||
        sources_[index].group_count) {
        return false;
    }
    for (size_t i = 0; i < groups.size(); ++i) {
        if (groups[i].bus > 3 || groups[i].id > 0x7ff) {
            return false;
        }
        for (size_t j = 0; j < i; ++j) {
            if (groups[i] == groups[j]) {
                return false;
            }
        }
        for (const auto& source : sources_) {
            if (std::find(source.groups.begin(),
                          source.groups.begin() + source.group_count,
                          groups[i]) !=
                source.groups.begin() + source.group_count) {
                return false;
            }
        }
    }
    auto& source = sources_[index];
    std::copy(groups.begin(), groups.end(), source.groups.begin());
    source.group_count = groups.size();
    source.timeout_ns = timeout_ns;
    return true;
}
bool EthercatTransmitter::HasCanProducers() const {
    return source_mask() != 0;
}
uint32_t EthercatTransmitter::source_mask() const {
    const std::lock_guard lock(mutex_);
    uint32_t result = 0;
    for (unsigned i = 0; i < 2; ++i) {
        if (sources_[i].group_count) {
            result |= 1U << i;
        }
    }
    return result;
}
uint32_t EthercatTransmitter::can_channels() const {
    const std::lock_guard lock(mutex_);
    uint32_t result = 0;
    for (const auto& source : sources_) {
        for (size_t i = 0; i < source.group_count; ++i) {
            result |= 1U << source.groups[i].bus;
        }
    }
    return result;
}
void EthercatTransmitter::SealRegistration() {
    const std::lock_guard lock(mutex_);
    registration_closed_ = true;
}
void EthercatTransmitter::Activate(uint64_t now_ns) {
    const std::lock_guard lock(mutex_);
    if (!started_ && !stopped_) {
        started_ = true;
        started_ns_ = now_ns;
        active_ = true;
    }
}
void EthercatTransmitter::RequestStop() {
    stopped_ = true;
    active_ = false;
}
void EthercatTransmitter::Fail(const std::string& reason) {
    const std::lock_guard lock(mutex_);
    if (error_.empty()) {
        error_ = reason;
    }
    stopped_ = true;
    active_ = false;
}
std::string EthercatTransmitter::error() const {
    const std::lock_guard lock(mutex_);
    return error_;
}
bool EthercatTransmitter::SubmitCan(CanProducer producer,
                                    std::span<const devices::CanFrame> frames,
                                    uint64_t sequence, uint64_t generated_ns) {
    const auto now = realtime::MonotonicTimeNs();
    const std::lock_guard lock(mutex_);
    const auto index = static_cast<size_t>(producer);
    if (!healthy() || index >= 2 || !sequence || generated_ns > now ||
        frames.size() > 16) {
        return false;
    }
    auto& source = sources_[index];
    if (!source.group_count || sequence <= source.sequence ||
        generated_ns < source.generated_ns ||
        now - generated_ns >= source.timeout_ns ||
        now - (source.batch.valid ? source.generated_ns : started_ns_) >=
            source.timeout_ns) {
        return false;
    }
    for (size_t i = 0; i < frames.size(); ++i) {
        const CanCommandGroup group{frames[i].bus, frames[i].id};
        if (std::find(source.groups.begin(),
                      source.groups.begin() + source.group_count,
                      group) == source.groups.begin() + source.group_count) {
            return false;
        }
        for (size_t j = 0; j < i; ++j) {
            if (frames[i].bus == frames[j].bus &&
                frames[i].id == frames[j].id) {
                return false;
            }
        }
    }
    source.sequence = sequence;
    source.generated_ns = generated_ns;
    source.batch.valid = true;
    source.batch.sequence = static_cast<uint32_t>(sequence);
    source.batch.count = frames.size();
    std::copy(frames.begin(), frames.end(), source.batch.frames.begin());
    return true;
}
bool EthercatTransmitter::ReadBatch(unsigned index, uint64_t now,
                                    Batch& batch) {
    const std::lock_guard lock(mutex_);
    batch = {};
    const auto& source = sources_.at(index);
    if (!source.group_count) {
        return true;
    }
    const auto time = source.batch.valid ? source.generated_ns : started_ns_;
    if (now < time || now - time >= source.timeout_ns) {
        return false;
    }
    batch = source.batch;
    batch.remaining_us =
        static_cast<uint32_t>((source.timeout_ns - (now - time)) / 1000);
    return batch.remaining_us != 0;
}
bool EthercatTransmitter::SubmitUart(uint8_t channel,
                                     std::span<const uint8_t> bytes,
                                     uint64_t generated_ns,
                                     uint64_t lifetime_ns) {
    const auto now = realtime::MonotonicTimeNs();
    const std::lock_guard lock(mutex_);
    if (!healthy() || channel < 1 || channel > 6 ||
        !capacity_.uart[channel - 1].tx_bytes || bytes.empty() ||
        bytes.size() > 512 || !lifetime_ns || lifetime_ns > 1000000000 ||
        generated_ns > now || now - generated_ns >= lifetime_ns) {
        return false;
    }
    auto& uart = uart_[channel - 1];
    if (uart.count == uart.queue.size()) {
        return false;
    }
    auto& message = uart.queue[(uart.head + uart.count) % uart.queue.size()];
    message = {};
    std::copy(bytes.begin(), bytes.end(), message.bytes.begin());
    message.size = static_cast<uint16_t>(bytes.size());
    message.generated_ns = generated_ns;
    message.lifetime_ns = lifetime_ns;
    ++uart.count;
    return true;
}
bool EthercatTransmitter::NextUart(unsigned port, uint32_t ack, bool ack_valid,
                                   uint64_t now, UartBlock& block) {
    const std::lock_guard lock(mutex_);
    block = {};
    auto& uart = uart_.at(port);
    if (ack_valid && (!uart.have_ack || ack != uart.last_ack)) {
        // Only acknowledge a member actually sent in the current bounded
        // window.
        size_t acknowledged = 0;
        for (size_t i = 0; i < uart.window_count; ++i) {
            const auto& pending = uart.window[(uart.window_head + i) % 4];
            if (pending.sent && pending.block.sequence == ack) {
                acknowledged = i + 1;
                break;
            }
        }
        if (!acknowledged && !(ack == 0 && !uart.have_ack)) {
            return false;
        }
        uart.window_head = (uart.window_head + acknowledged) % 4;
        uart.window_count -= acknowledged;
        uart.cursor = 0;
        uart.last_ack = ack;
        uart.have_ack = true;
    }
    while (uart.count && uart.window_count < 4) {
        auto& message = uart.queue[uart.head];
        if (now < message.generated_ns) {
            return false;
        }
        if (now - message.generated_ns >= message.lifetime_ns) {
            if (message.cursor) {
                return false;
            }
            uart.head = (uart.head + 1) % uart.queue.size();
            --uart.count;
            continue;
        }
        auto& pending = uart.window[(uart.window_head + uart.window_count) % 4];
        pending = {};
        const auto count = std::min<size_t>(message.size - message.cursor,
                                            capacity_.uart[port].tx_bytes);
        if (!count) {
            break;
        }
        pending.block.valid = true;
        pending.block.size = static_cast<uint16_t>(count);
        pending.block.sequence = uart.next_sequence++;
        std::copy_n(message.bytes.begin() + message.cursor, count,
                    pending.block.bytes.begin());
        pending.generated_ns = message.generated_ns;
        pending.lifetime_ns = message.lifetime_ns;
        message.cursor += count;
        if (message.cursor == message.size) {
            uart.head = (uart.head + 1) % uart.queue.size();
            --uart.count;
        }
        ++uart.window_count;
    }
    for (size_t i = 0; i < uart.window_count; ++i) {
        const auto& pending = uart.window[(uart.window_head + i) % 4];
        if (now < pending.generated_ns ||
            now - pending.generated_ns >= pending.lifetime_ns ||
            (pending.sent && now - pending.first_sent_ns >= 20000000)) {
            return false;
        }
    }
    if (uart.window_count) {
        auto& pending =
            uart.window[(uart.window_head + uart.cursor % uart.window_count) %
                        4];
        if (!pending.sent) {
            pending.sent = true;
            pending.first_sent_ns = now;
        }
        block = pending.block;
        uart.cursor = (uart.cursor + 1) % uart.window_count;
    }
    return true;
}
}  // namespace dust_control::host
