/* SPDX-License-Identifier: Apache-2.0 */
#include "session.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <mutex>
#include <stdexcept>

#include "wire.hpp"
#include "pdo_schema.hpp"

namespace dust_control::host {
namespace w = ethercat::wire;
namespace pdo = dust::pdo;
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
    if (!w::U32(p + pdo::tx::header::boot_id) ||
        !(w::U32(p + pdo::tx::header::capability_mask) &
          w::kFirmwareProtection)) {
        return FailInput("EtherCAT boot/protection capability mismatch");
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
    if ((w::U32(p + pdo::tx::header::capability_mask) & required) != required) {
        return FailInput("EtherCAT configured channel capability missing");
    }
    const auto boot = w::U32(p + pdo::tx::header::boot_id);
    if (boot_id_ && boot != boot_id_) {
        return FailInput("EtherCAT slave restarted; new session required");
    }
    boot_id_ = boot;
    if (p[pdo::tx::header::fault_flags] ||
        p[pdo::tx::header::business_state] == w::kProtected) {
        return FailInput("EtherCAT slave reports protection/fault");
    }
    const auto state = p[pdo::tx::header::business_state];
    if (state != w::kReady && state != w::kActive) {
        return FailInput("Unexpected EtherCAT business state");
    }
    if (state == w::kActive &&
        w::U32(p + pdo::tx::header::accepted_session_id) != uint32_t(session)) {
        return FailInput("EtherCAT session mismatch");
    }
    if (active_ && state != w::kActive) {
        return FailInput("EtherCAT business state regressed");
    }
    active_ = state == w::kActive;
    const auto sequence = w::U32(p + pdo::tx::header::publish_sequence);
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
    const auto slave_now = w::U32(p + pdo::tx::header::slave_now_us);
    for (unsigned bus = 0; bus < 4; ++bus) {
        if (!can_[bus]) {
            continue;
        }
        const auto* header = p + *can_[bus];
        const auto count = header[pdo::tx::can::frame_count];
        if (count > capacity_.can_rx_slots[bus]) {
            return FailInput("EtherCAT CAN feedback count exceeds capacity");
        }
        auto& seq = can_sequence_[bus];
        const auto current = w::U32(header + pdo::tx::can::batch_sequence);
        if (seq.present && current == seq.value) {
            continue;
        }
        if (seq.present && !w::Newer(current, seq.value)) {
            return FailInput("EtherCAT CAN batch reordered");
        }
        seq = {current, true};
        const bool gap =
            can_drops_[bus] != w::U32(header + pdo::tx::can::cumulative_drops);
        can_drops_[bus] = w::U32(header + pdo::tx::can::cumulative_drops);
        if (gap && options_.decode_motors) motors_.Reset();
        std::array<uint32_t, 32> seen{};
        for (unsigned i = 0; i < count; ++i) {
            const auto* frame = header + pdo::tx::can::header_size +
                                pdo::tx::can::frame_stride * i;
            const auto id = w::U16(frame + pdo::tx::can::frame_id);
            if (id > 0x7ff || frame[pdo::tx::can::frame_length] > 8)
                return FailInput("Invalid EtherCAT CAN feedback frame");
            for (unsigned j = 0; j < i; ++j)
                if (seen[j] == id) return FailInput("Duplicate CAN latest ID");
            seen[i] = id;
            auto& slots = can_latest_[bus];
            auto slot = std::find_if(slots.begin(), slots.end(),
                [id](const CanLatest& x) { return x.sequence.present && x.id == id; });
            if (slot == slots.end()) {
                slot = std::find_if(slots.begin(), slots.end(),
                    [](const CanLatest& x) { return !x.sequence.present; });
                if (slot == slots.end()) return FailInput("CAN latest slots exhausted");
                slot->id = id;
            }
            const uint32_t received =
                w::U32(frame + pdo::tx::can::frame_bus_sequence);
            if (slot->sequence.present) {
                if (received == slot->sequence.value) {
                    if (!std::equal(slot->frame.begin(), slot->frame.end(), frame))
                        return FailInput("CAN latest sequence reused with different data");
                    continue;
                }
                if (!w::Newer(received, slot->sequence.value))
                    return FailInput("CAN latest sequence reordered");
            }
            slot->sequence = {received, true};
            std::copy_n(frame, slot->frame.size(), slot->frame.begin());
            const auto sample =
                SampleUs(slave_now,
                         w::U32(frame + pdo::tx::can::frame_capture_time_us),
                         now_ns, layout_.cycle_time_us);
            if (options_.decode_motors) {
                motors_.Consume(bus, id,
                                {frame + pdo::tx::can::frame_data,
                                 frame[pdo::tx::can::frame_length]},
                                sample, sample != 0);
            }
        }
    }
    for (unsigned port = 0; port < 6; ++port) {
        if (!uart_tx_[port]) {
            continue;
        }
        const auto* header = p + *uart_tx_[port];
        const auto length = w::U16(header + pdo::tx::uart::length);
        const auto flags = header[pdo::tx::uart::flags];
        if (length > capacity_.uart[port].rx_bytes || (flags & ~uint16_t(7))) {
            return FailInput("Invalid EtherCAT UART block");
        }
        uart_peer_ack_[port] =
            w::U32(header + pdo::tx::uart::peer_ack_sequence);
        uart_peer_ack_valid_[port] = flags & w::kAckValid;
        if (!(flags & w::kDataValid)) {
            if (length) {
                return FailInput("UART length without data_valid");
            }
            continue;
        }
        auto& seq = uart_sequence_[port];
        const auto current = w::U32(header + pdo::tx::uart::block_sequence);
        if (seq.present && !w::Newer(current, seq.value)) {
            continue;
        }
        const bool gap = flags & w::kDiscontinuity;
        if (!gap && current != (seq.present ? seq.value + 1U : 1U)) {
            continue;
        }  // Request retransmission through cumulative ACK.
        const auto sample =
            SampleUs(slave_now, w::U32(header + pdo::tx::uart::capture_time_us),
                     now_ns, layout_.cycle_time_us);
        if (!sample) {
            return FailInput("EtherCAT UART sample too old");
        }
        if (gap) {
            Discontinuity(port);
        }
        const std::span<const uint8_t> bytes(header + pdo::tx::uart::header_size,
                                             length);
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
        const auto seq = w::U32(raw + pdo::tx::imu::sample_sequence);
        if ((raw[pdo::tx::imu::flags] & 1) &&
            (!have_imu_ || w::Newer(seq, imu_sequence_))) {
            const auto sample =
                SampleUs(slave_now,
                         w::U32(raw + pdo::tx::imu::capture_time_us), now_ns,
                         layout_.cycle_time_us);
            const float temperature =
                std::bit_cast<float>(w::U32(raw + pdo::tx::imu::temperature_c));
            if (!sample || !std::isfinite(temperature)) {
                return FailInput("Invalid EtherCAT onboard IMU sample");
            }
            std::array<uint8_t, 25> data{};
            std::copy_n(raw + pdo::tx::imu::spi_bytes, 25, data.begin());
            onboard_imu_.Consume(data, temperature, seq, sample);
            imu_sequence_ = seq;
            have_imu_ = true;
        }
    }
    std::array<uint8_t, 28> diagnostics{};
    std::copy_n(p + diagnostics_offset_, diagnostics.size(),
                diagnostics.begin());
    diagnostics_.Publish(diagnostics);
    const auto* diag = p + diagnostics_offset_;
    status_reply_.Publish(
        {w::U32(diag + pdo::tx::diagnostics::status_reply_sequence),
         std::bit_cast<int32_t>(w::U32(diag + pdo::tx::diagnostics::status_result)),
         {w::U32(diag + pdo::tx::diagnostics::status_value0),
          w::U32(diag + pdo::tx::diagnostics::status_value1),
          w::U32(diag + pdo::tx::diagnostics::status_value2)}});
    remote_.PollTimeout(now_ns / 1000);
    return true;
}
bool EthercatSession::Encode(std::span<uint8_t> image, uint64_t now_ns,
                             uint64_t session, uint32_t exchange,
                             uint32_t flags) {
    if (image.size() != layout_.rx.bytes) {
        return FailInput("EtherCAT RxPDO length mismatch");
    }
    if ((flags != w::kDiscover && flags != w::kArm && flags != w::kStop) ||
        (transmitter_.source_mask() & ~3U)) {
        return FailInput("Invalid EtherCAT control flags or source mask");
    }
    std::fill(image.begin(), image.end(), 0);
    auto* p = image.data();
    w::Put32(p + pdo::rx::header::session_id, uint32_t(session));
    w::Put32(p + pdo::rx::header::expected_boot_id, boot_id_);
    w::Put32(p + pdo::rx::header::exchange_sequence, exchange);
    p[pdo::rx::header::control_flags] = uint8_t(flags);
    p[pdo::rx::header::active_source_mask] =
        uint8_t(transmitter_.source_mask());
    {
        const std::lock_guard lock(status_mutex_);
        w::Put16(p + pdo::rx::header::status_request_sequence,
                 uint16_t(status_request_));
        p[pdo::rx::header::status_target] = status_target_;
        p[pdo::rx::header::status_channel] = status_channel_;
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
            w::Put32(out + pdo::rx::can::source_sequence, batch.sequence);
            w::Put32(out + pdo::rx::can::remaining_valid_us,
                     batch.remaining_us);
            out[pdo::rx::can::flags] = 1;
            out[pdo::rx::can::frame_count] = static_cast<uint8_t>(batch.count);
            out[pdo::rx::can::source_id] = source;
            for (size_t i = 0; i < batch.count; ++i) {
                auto* frame = out + pdo::rx::can::header_size +
                              pdo::rx::can::frame_stride * i;
                frame[pdo::rx::can::frame_channel] = batch.frames[i].bus;
                frame[pdo::rx::can::frame_length] = 8;
                w::Put16(frame + pdo::rx::can::frame_id,
                         uint16_t(batch.frames[i].id));
                std::copy(batch.frames[i].bytes.begin(),
                          batch.frames[i].bytes.end(),
                          frame + pdo::rx::can::frame_data);
            }
        }
    }
    for (unsigned port = 0; port < 6; ++port) {
        if (!uart_rx_[port]) {
            continue;
        }
        auto* out = p + *uart_rx_[port];
        uint8_t uart_flags = 0;
        if (uart_sequence_[port].present) {
            w::Put32(out + pdo::rx::uart::peer_ack_sequence,
                     uart_sequence_[port].value);
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
                w::Put32(out + pdo::rx::uart::block_sequence, block.sequence);
                w::Put16(out + pdo::rx::uart::length, block.size);
                uart_flags |= w::kDataValid;
                std::copy_n(block.bytes.begin(), block.size,
                            out + pdo::rx::uart::header_size);
            }
        }
        out[pdo::rx::uart::flags] = uart_flags;
    }
    return true;
}
}  // namespace dust_control::host
