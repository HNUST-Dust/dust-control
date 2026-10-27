/* SPDX-License-Identifier: Apache-2.0 */
#include "session.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <mutex>
#include <stdexcept>

#include "wire.hpp"

namespace dust_control::host {
namespace w = ethercat::wire;
namespace {
uint32_t Offset(const ethercat::ProcessImageLayout& image,
                const std::string& name) {
    for (const auto& mapping : image.mappings) {
        for (const auto& entry : mapping.entries) {
            if (entry.name == name) {
                return entry.offset;
            }
        }
    }
    throw std::invalid_argument("Missing PDO field: " + name);
}
uint64_t SampleUs(uint32_t now, uint32_t capture, uint64_t arrival_ns,
                  uint32_t cycle_us) {
    const auto age = uint32_t(now - capture);
    // Conservatively charge one complete process cycle for transport delay.
    const uint64_t total_age = uint64_t(age) + cycle_us;
    if (age >= kCommandTimeoutUs || arrival_ns / 1000 < total_age) {
        return 0;
    }
    return arrival_ns / 1000 - total_age;
}
}  // namespace
void UartSensorBindings::Validate() const {
    std::array<bool, 7> used{};
    for (auto port : {remote, hi91, referee}) {
        if (port > 6 || (port && used[port])) {
            throw std::invalid_argument(
                "UART sensor channels must be unique in 1..6 (0 disables)");
        }
        used[port] = port != 0;
    }
}
EthercatSession::EthercatSession(const ethercat::AdapterPdoConfig& capacity,
                                 const ethercat::AdapterPdoLayout& layout,
                                 EthercatDeviceOptions options,
                                 const devices::MotorConfig& motors,
                                 const devices::ImuConfig& imu)
    : capacity_(capacity),
      layout_(layout),
      options_(options),
      transmitter_(capacity),
      motors_(motors),
      imu_(options.strict_hi91_crc, imu) {
    options_.uart_sensors.Validate();
    for (auto port : {options_.uart_sensors.remote, options_.uart_sensors.hi91,
                      options_.uart_sensors.referee}) {
        if (port && !capacity_.uart[port - 1].rx_bytes) {
            throw std::invalid_argument(
                "Sensor binding requires UART RX capacity");
        }
    }
    for (unsigned i = 0; i < 2; ++i) {
        if (capacity_.can_tx_slots[i]) {
            sources_[i] = Offset(layout_.rx, i ? "can.gimbal.source_sequence"
                                               : "can.chassis.source_sequence");
        }
    }
    for (unsigned bus = 0; bus < 4; ++bus) {
        if (capacity_.can_rx_slots[bus]) {
            can_[bus] = Offset(layout_.tx,
                               "can" + std::to_string(bus) + ".batch_sequence");
        }
    }
    for (size_t i = 0; i < motors.present.size(); ++i) {
        if (options_.decode_motors && motors.present[i] &&
            !capacity_.can_rx_slots.at(motors.addresses[i].bus)) {
            throw std::invalid_argument(
                "Motor binding requires CAN RX capacity");
        }
    }
    for (unsigned port = 0; port < 6; ++port) {
        if (capacity_.uart[port].tx_bytes || capacity_.uart[port].rx_bytes) {
            const auto field =
                "uart" + std::to_string(port + 1) + ".block_sequence";
            uart_rx_[port] = Offset(layout_.rx, field);
            uart_tx_[port] = Offset(layout_.tx, field);
        }
    }
    if (capacity_.onboard_imu) {
        imu_offset_ = Offset(layout_.tx, "imu.sample_sequence");
    }
    diagnostics_offset_ = Offset(layout_.tx, "diagnostics.stop_reason");
}
uint32_t EthercatSession::RequestStatus(uint8_t target, uint8_t channel,
                                        uint8_t page) {
    if (target > 2 || page > 63 || (target == 0 && channel != 0) ||
        (target == 1 && channel > 3) ||
        (target == 2 && (channel < 1 || channel > 6))) {
        throw std::invalid_argument("Invalid EtherCAT status request");
    }
    const std::lock_guard lock(status_mutex_);
    if (++status_request_ == 0) {
        ++status_request_;
    }
    status_target_ = static_cast<uint8_t>((page << 2) | target);
    status_channel_ = channel;
    return status_request_;
}
bool EthercatSession::FailInput(const char* reason) {
    Fail(reason);
    return false;
}
void EthercatSession::Discontinuity(unsigned port) {
    const auto channel = port + 1;
    if (channel == options_.uart_sensors.remote) {
        remote_.Discontinuity();
    }
    if (channel == options_.uart_sensors.hi91) {
        imu_.Discontinuity();
    }
    if (channel == options_.uart_sensors.referee) {
        referee_.Discontinuity();
    }
}
bool EthercatSession::Receive(std::span<const uint8_t> image, uint64_t now_ns,
                              uint64_t session) {
    if (image.size() != layout_.tx.bytes) {
        return FailInput("EtherCAT TxPDO length mismatch");
    }
    const auto* p = image.data();
    if (w::U32(p) != w::kMagic || w::U16(p + 4) != w::kMajor ||
        w::U16(p + 6) != w::kMinor || !w::U64(p + 8) ||
        w::U32(p + 32) != layout_.layout_signature ||
        !(w::U32(p + 36) & w::kFirmwareProtection)) {
        return FailInput("EtherCAT ABI/layout/protection capability mismatch");
    }
    uint32_t required = w::kFirmwareProtection;
    uint32_t channels = transmitter_.can_channels();
    for (unsigned bus = 0; bus < 4; ++bus) {
        if (capacity_.can_rx_slots[bus]) {
            channels |= 1U << bus;
        }
    }
    if (channels || transmitter_.source_mask()) {
        required |= 1;
    }
    required |= channels << 16;
    if (capacity_.onboard_imu) {
        required |= 8;
    }
    for (unsigned port = 1; port <= 6; ++port) {
        if (capacity_.uart[port - 1].rx_bytes ||
            capacity_.uart[port - 1].tx_bytes) {
            required |= 1U << (port < 3 ? port : port + 1);
        }
    }
    if ((w::U32(p + 36) & required) != required) {
        return FailInput("EtherCAT configured channel capability missing");
    }
    if (w::U32(p + 60)) {
        return FailInput("EtherCAT reserved header bits set");
    }
    const auto boot = w::U64(p + 8);
    if (boot_id_ && boot != boot_id_) {
        return FailInput("EtherCAT slave restarted; new session required");
    }
    boot_id_ = boot;
    if (w::U32(p + 44) || w::U32(p + 40) == w::kProtected) {
        return FailInput("EtherCAT slave reports protection/fault");
    }
    const auto state = w::U32(p + 40);
    if (state != w::kReady && state != w::kActive) {
        return FailInput("Unexpected EtherCAT business state");
    }
    if (state == w::kActive && w::U64(p + 16) != session) {
        return FailInput("EtherCAT session mismatch");
    }
    if (active_ && state != w::kActive) {
        return FailInput("EtherCAT business state regressed");
    }
    active_ = state == w::kActive;
    const auto sequence = w::U32(p + 24);
    if (have_publish_ && !w::Newer(sequence, publish_sequence_)) {
        if (sequence != publish_sequence_ ||
            now_ns - last_publish_ns_ >= 20000000) {
            return FailInput("EtherCAT feedback stream stale/reordered");
        }
        remote_.PollTimeout(now_ns / 1000);
        return true;
    }
    have_publish_ = true;
    publish_sequence_ = sequence;
    last_publish_ns_ = now_ns;
    const auto slave_now = w::U32(p + 28);
    for (unsigned bus = 0; bus < 4; ++bus) {
        if (!can_[bus]) {
            continue;
        }
        const auto* header = p + *can_[bus];
        const auto count = w::U16(header + 8);
        if (count > capacity_.can_rx_slots[bus]) {
            return FailInput("EtherCAT CAN feedback count exceeds capacity");
        }
        if (w::U16(header + 10) || w::U32(header + 12)) {
            return FailInput("EtherCAT CAN reserved flags set");
        }
        auto& seq = can_sequence_[bus];
        const auto current = w::U32(header);
        if (seq.present && current == seq.value) {
            continue;
        }
        if (seq.present && !w::Newer(current, seq.value)) {
            return FailInput("EtherCAT CAN batch reordered");
        }
        seq = {current, true};
        bool gap = can_drops_[bus] != w::U32(header + 4);
        can_drops_[bus] = w::U32(header + 4);
        auto& frame_sequence = can_frame_sequence_[bus];
        for (unsigned i = 0; i < count; ++i) {
            const auto received = w::U32(header + 16 + 24 * i + 4);
            if (frame_sequence.present &&
                !w::Newer(received, frame_sequence.value)) {
                return FailInput("EtherCAT CAN event reordered");
            }
            gap = gap || (frame_sequence.present &&
                          received != frame_sequence.value + 1U);
            frame_sequence = {received, true};
        }
        // Lost intermediate angles must not silently continue the old unwrap
        // state.
        if (gap && options_.decode_motors) {
            motors_.Reset();
        }
        for (unsigned i = 0; i < count; ++i) {
            const auto* frame = header + 16 + 24 * i;
            if (w::U32(frame) > 0x7ff || frame[12] > 8 || frame[13] ||
                w::U16(frame + 14)) {
                return FailInput("Invalid EtherCAT CAN feedback frame");
            }
            const auto sample = SampleUs(slave_now, w::U32(frame + 8), now_ns,
                                         layout_.cycle_time_us);
            if (options_.decode_motors) {
                motors_.Consume(bus, w::U32(frame), {frame + 16, frame[12]},
                                sample, sample != 0);
            }
        }
    }
    for (unsigned port = 0; port < 6; ++port) {
        if (!uart_tx_[port]) {
            continue;
        }
        const auto* header = p + *uart_tx_[port];
        const auto length = w::U16(header + 12), flags = w::U16(header + 14);
        if (length > capacity_.uart[port].rx_bytes || (flags & ~uint16_t(7))) {
            return FailInput("Invalid EtherCAT UART block");
        }
        uart_peer_ack_[port] = w::U32(header + 4);
        uart_peer_ack_valid_[port] = flags & w::kAckValid;
        if (!(flags & w::kDataValid)) {
            if (length) {
                return FailInput("UART length without data_valid");
            }
            continue;
        }
        auto& seq = uart_sequence_[port];
        const auto current = w::U32(header);
        if (seq.present && !w::Newer(current, seq.value)) {
            continue;
        }
        const bool gap = flags & w::kDiscontinuity;
        if (!gap && current != (seq.present ? seq.value + 1U : 1U)) {
            continue;
        }  // Request retransmission through cumulative ACK.
        const auto sample = SampleUs(slave_now, w::U32(header + 8), now_ns,
                                     layout_.cycle_time_us);
        if (!sample) {
            return FailInput("EtherCAT UART sample too old");
        }
        if (gap) {
            Discontinuity(port);
        }
        const std::span<const uint8_t> bytes(header + 16, length);
        UartReceiveChunk chunk{};
        std::copy(bytes.begin(), bytes.end(), chunk.bytes.begin());
        chunk.size = length;
        chunk.sequence = current;
        chunk.receive_us = sample;
        chunk.discontinuity = gap;
        uart_queue_.PublishUartChunk(port + 1, chunk);
        if (port + 1 == options_.uart_sensors.remote) {
            remote_.Consume(bytes, now_ns / 1000);
        }
        if (port + 1 == options_.uart_sensors.hi91) {
            imu_.Consume(bytes, sample);
        }
        if (port + 1 == options_.uart_sensors.referee) {
            referee_.Consume(bytes, sample);
        }
        seq = {current, true};
    }
    if (imu_offset_) {
        const auto* raw = p + *imu_offset_;
        const auto seq = w::U32(raw);
        if ((w::U32(raw + 8) & 1) &&
            (!have_imu_ || w::Newer(seq, imu_sequence_))) {
            const auto sample = SampleUs(slave_now, w::U32(raw + 4), now_ns,
                                         layout_.cycle_time_us);
            const float temperature = std::bit_cast<float>(w::U32(raw + 12));
            if (!sample || !std::isfinite(temperature)) {
                return FailInput("Invalid EtherCAT onboard IMU sample");
            }
            std::array<uint8_t, 25> data{};
            std::copy_n(raw + 16, 25, data.begin());
            onboard_imu_.Consume(data, temperature, seq, sample);
            imu_sequence_ = seq;
            have_imu_ = true;
        }
    }
    std::array<uint8_t, 224> diagnostics{};
    std::copy_n(p + diagnostics_offset_, diagnostics.size(),
                diagnostics.begin());
    diagnostics_.Publish(diagnostics);
    const auto* reply = p + diagnostics_offset_ + 208;
    status_reply_.Publish(
        {w::U32(p + 56),
         std::bit_cast<int32_t>(w::U32(reply)),
         {w::U32(reply + 4), w::U32(reply + 8), w::U32(reply + 12)}});
    remote_.PollTimeout(now_ns / 1000);
    return true;
}
bool EthercatSession::Encode(std::span<uint8_t> image, uint64_t now_ns,
                             uint64_t session, uint32_t exchange,
                             uint32_t flags) {
    if (image.size() != layout_.rx.bytes) {
        return FailInput("EtherCAT RxPDO length mismatch");
    }
    std::fill(image.begin(), image.end(), 0);
    auto* p = image.data();
    w::Put32(p, w::kMagic);
    w::Put16(p + 4, w::kMajor);
    w::Put16(p + 6, w::kMinor);
    w::Put64(p + 8, session);
    w::Put64(p + 16, boot_id_);
    w::Put32(p + 24, exchange);
    w::Put32(p + 28, flags);
    w::Put32(p + 32, layout_.layout_signature);
    w::Put32(p + 36, transmitter_.source_mask());
    w::Put32(p + 40, layout_.cycle_time_us);
    {
        const std::lock_guard lock(status_mutex_);
        w::Put32(p + 44, status_request_);
        p[48] = status_target_;
        p[49] = status_channel_;
    }
    if (flags & w::kStop) {
        return true;
    }
    if (transmitter_.healthy()) {
        for (unsigned source = 0; source < 2; ++source) {
            if (!sources_[source]) {
                continue;
            }
            EthercatTransmitter::Batch batch;
            if (!transmitter_.ReadBatch(source, now_ns, batch)) {
                return FailInput("EtherCAT CAN control source expired");
            }
            if (!batch.valid) {
                continue;
            }
            auto* out = p + *sources_[source];
            w::Put32(out, batch.sequence);
            w::Put32(out + 4, batch.remaining_us);
            w::Put16(out + 8, 1);
            out[10] = static_cast<uint8_t>(batch.count);
            out[11] = source;
            for (size_t i = 0; i < batch.count; ++i) {
                auto* frame = out + 16 + 16 * i;
                frame[0] = batch.frames[i].bus;
                frame[1] = 8;
                w::Put32(frame + 4, batch.frames[i].id);
                std::copy(batch.frames[i].bytes.begin(),
                          batch.frames[i].bytes.end(), frame + 8);
            }
        }
    }
    for (unsigned port = 0; port < 6; ++port) {
        if (!uart_rx_[port]) {
            continue;
        }
        auto* out = p + *uart_rx_[port];
        uint16_t uart_flags = 0;
        if (uart_sequence_[port].present) {
            w::Put32(out + 4, uart_sequence_[port].value);
            uart_flags |= w::kAckValid;
        }
        if (transmitter_.healthy() && capacity_.uart[port].tx_bytes) {
            EthercatTransmitter::UartBlock block;
            if (!transmitter_.NextUart(port, uart_peer_ack_[port],
                                       uart_peer_ack_valid_[port], now_ns,
                                       block)) {
                return FailInput("EtherCAT UART window/ack/deadline failure");
            }
            if (block.valid) {
                w::Put32(out, block.sequence);
                w::Put16(out + 12, block.size);
                uart_flags |= w::kDataValid;
                std::copy_n(block.bytes.begin(), block.size, out + 16);
            }
        }
        w::Put16(out + 14, uart_flags);
    }
    return true;
}
}  // namespace dust_control::host
