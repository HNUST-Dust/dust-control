/* SPDX-License-Identifier: Apache-2.0 */
#include <algorithm>
#include <span>

#include "chassis/runtime/realtime_loop.hpp"
#include "usb_link.hpp"
#include "usb_transmitter.hpp"

namespace dust_control::host {
// 回调在检查实际长度后进入这里；握手、确认和业务报告按会话顺序处理。
void UsbLink::HandleReport(const dust_control::wire::UsbInPacket& packet) {
    if (dust_control::wire::U32(packet.data()) == dust_control::wire::kMagic &&
        packet[8] != dust_control::wire::kVersion) {
        FailSession("Adapter protocol version mismatch (v4 required)");
        return;
    }

    if (dust_control::wire::U32(packet.data()) == dust_control::wire::kMagic &&
        packet[8] == dust_control::wire::kVersion &&
        dust_control::wire::U32(packet.data() + 12) == session_) {
        last_receive_us_ = dust_control::realtime::MonotonicTimeNs() / 1000;
        if (packet[9] == uint8_t(dust_control::wire::Kind::Ready)) {
            HandleReady(packet);
        } else if (packet[9] == uint8_t(dust_control::wire::Kind::Result)) {
            HandleResult(packet);
        } else if (packet[9] == uint8_t(dust_control::wire::Kind::Status)) {
            // 显式状态查询结果不进入传感器解码流程。
        } else if (ready_received_) {
            const uint64_t arrival_ns =
                dust_control::realtime::MonotonicTimeNs();
            if (!ReceiveSensorReport(packet, arrival_ns / 1000)) {
                FailSession("Malformed bridge sensor packet");
            }
        }
    }
}

void UsbLink::HandleReady(const dust_control::wire::UsbInPacket& packet) {
    if (ready_received_) {
        FailSession("Unexpected bridge session restart");
        return;
    }
    dust_control::wire::ReadyReport report{};
    if (!dust_control::wire::Decode(packet, report)) {
        FailSession("Malformed adapter Ready report");
        return;
    }
    if (report.request_bytes != dust_control::wire::kOutBytes ||
        report.max_report_bytes != dust_control::wire::kInBytes ||
        (report.capabilities & 7) != 7) {
        FailSession("Bridge CAN/remote/HI91 capability missing");
        return;
    }
    const auto channels = report.can_channel_mask;
    if (options_.decode_motors) {
        for (size_t i = 0; i < motor_config_.addresses.size(); ++i) {
            if (!motor_config_.present[i]) {
                continue;
            }
            const auto& motor = motor_config_.addresses[i];
            if (!(channels & (1U << motor.bus))) {
                FailSession("Adapter CAN channel capability missing");
                return;
            }
        }
    }
    if (report.max_can_frames < 6) {
        FailSession("Adapter batch/safety capacity missing");
        return;
    }
    if (!(report.protection_flags &
          dust_control::wire::kFirmwareCanProtection)) {
        FailSession("Adapter firmware CAN protection profile missing");
        return;
    }
    max_lease_us_ = report.max_lease_us;
    capabilities_ = report.capabilities;
    ResetSensors();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ready_received_ = true;
    }
    ready_cv_.notify_all();
}

void UsbLink::HandleResult(const dust_control::wire::UsbInPacket& packet) {
    dust_control::wire::RequestResultReport report{};
    if (!dust_control::wire::Decode(packet, report)) {
        FailSession("Malformed adapter control acknowledgement");
        return;
    }
    const auto sequence = report.request_sequence;
    if (sequence == awaited_request_sequence_.load()) {
        const auto status = report.result_code;
        if (uint8_t(report.request_kind) != awaited_request_kind_.load() ||
            (!status && report.request_kind == dust_control::wire::Kind::Arm &&
             !report.armed)) {
            FailSession("Malformed adapter control acknowledgement");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            result_code_ = report.result_code;
            acknowledged_request_sequence_ = sequence;
        }
        ready_cv_.notify_all();
    } else if (report.result_code) {
        FailSession("Adapter rejected a data request");
        return;
    }
}

namespace {
// MCU 时间只用于计算采集到上报的年龄；不假设两端时钟已同步。
bool SampleTime(uint32_t transmitted_us, uint32_t captured_us,
                uint64_t arrival_us, uint64_t& sample_us) {
    const uint32_t age = uint32_t(transmitted_us - captured_us);
    if (age > kCommandTimeoutUs || arrival_us < age) {
        sample_us = 0;
        return false;
    }
    sample_us = arrival_us - age;
    return true;
}
}  // namespace

void UsbLink::ResetSensors() {
    motor_decoder_.Reset();
    wfly_sbus_decoder_.Reset();
    hi91_decoder_.Reset();
    referee_decoder_.Reset();
    onboard_imu_.Reset();
    uart_sequences_ = {};
    uart_queue_.ResetUartChunks();
}

bool UsbLink::ReceiveSensorReport(const dust_control::wire::UsbInPacket& packet,
                                  uint64_t arrival_us) {
    // 实际传输长度和会话已由 CompleteReceive 检查。
    if (!dust_control::wire::HasHeader(packet.data(), packet.size())) {
        return false;
    }
    switch (dust_control::wire::Kind(packet[9])) {
        case dust_control::wire::Kind::CanReceiveReport: {
            dust_control::wire::CanReceiveReport report{};
            if (!dust_control::wire::Decode(packet, report)) {
                return false;
            }
            ReceiveCanReport(report, arrival_us);
            break;
        }
        case dust_control::wire::Kind::UartReceiveReport: {
            dust_control::wire::UartReceiveReport report{};
            if (!dust_control::wire::Decode(packet, report) ||
                !ReceiveUartReport(report, arrival_us)) {
                return false;
            }
            break;
        }
        case dust_control::wire::Kind::ImuSampleReport: {
            dust_control::wire::ImuSampleReport report{};
            if (!dust_control::wire::Decode(packet, report) ||
                !ReceiveOnboardImuReport(report, arrival_us)) {
                return false;
            }
            break;
        }
        default:
            return false;
    }
    return true;
}

void UsbLink::ReceiveCanReport(
    const dust_control::wire::CanReceiveReport& report, uint64_t arrival_us) {
    if (!options_.decode_motors) {
        return;
    }
    for (unsigned i = 0; i < report.frame_count; ++i) {
        const auto& received = report.frames[i];
        const auto& frame = received.frame;
        if (frame.flags) {
            continue;
        }
        uint64_t sample_us = 0;
        const bool timestamp_valid =
            SampleTime(report.header.transmit_timestamp_us,
                       received.receive_timestamp_us, arrival_us, sample_us);
        motor_decoder_.Consume(frame.channel, frame.id,
                               {frame.data, frame.length}, sample_us,
                               timestamp_valid);
    }
}

bool UsbLink::ReceiveUartReport(
    const dust_control::wire::UartReceiveReport& report, uint64_t arrival_us) {
    uint64_t sample_us = 0;
    if (!SampleTime(report.header.transmit_timestamp_us,
                    report.receive_timestamp_us, arrival_us, sample_us)) {
        return false;
    }
    auto& sequence = uart_sequences_[report.channel - 1];
    const bool discontinuity =
        sequence.present && report.block_sequence != sequence.sequence + 1;
    sequence = {report.block_sequence, true};
    const std::span<const uint8_t> bytes(report.bytes.data(),
                                         report.byte_count);
    UartReceiveChunk chunk{};
    std::copy(bytes.begin(), bytes.end(), chunk.bytes.begin());
    chunk.size = report.byte_count;
    chunk.sequence = report.block_sequence;
    chunk.receive_us = sample_us;
    chunk.discontinuity = discontinuity;
    uart_queue_.PublishUartChunk(report.channel, chunk);
    if (report.channel == options_.uart_sensors.remote) {
        if (discontinuity) {
            wfly_sbus_decoder_.Discontinuity();
        }
        // 遥控丢帧计时继续使用主机实际接收时间。
        wfly_sbus_decoder_.Consume(bytes, arrival_us);
    } else if (report.channel == options_.uart_sensors.hi91) {
        if (discontinuity) {
            hi91_decoder_.Discontinuity();
        }
        hi91_decoder_.Consume(bytes, sample_us);
    } else if (report.channel == options_.uart_sensors.referee) {
        if (discontinuity) {
            referee_decoder_.Discontinuity();
        }
        referee_decoder_.Consume(bytes, sample_us);
    }
    return true;
}

bool UsbLink::ReceiveOnboardImuReport(
    const dust_control::wire::ImuSampleReport& report, uint64_t arrival_us) {
    uint64_t sample_us = 0;
    if (!SampleTime(report.header.transmit_timestamp_us,
                    report.sample_timestamp_us, arrival_us, sample_us)) {
        return false;
    }
    onboard_imu_.Consume(report.spi_bytes, report.temperature_c,
                         report.sample_sequence, sample_us);
    return true;
}
}  // namespace dust_control::host
