/* SPDX-License-Identifier: Apache-2.0 */
#include <algorithm>
#include <span>

#include "protocol/usb_report_decoder.hpp"
#include "usb_link.hpp"

namespace wbr::host {
namespace {
// MCU 时间只用于计算采集到上报的年龄；不假设两端时钟已同步。
bool SampleTime(uint32_t transmitted_us, uint32_t captured_us,
                uint64_t arrival_us, uint64_t& sample_us) {
    const uint32_t age = uint32_t(transmitted_us - captured_us);
    if (age > bridge::kCommandTimeoutUs || arrival_us < age) {
        sample_us = 0;
        return false;
    }
    sample_us = arrival_us - age;
    return true;
}
}  // namespace

void UsbLink::ResetSensors() {
    motor_decoder_.Reset();
    sbus_decoder_.Reset();
    hi91_decoder_.Reset();
    referee_decoder_.Reset();
    onboard_imu_.Reset();
    uart_sequences_ = {};
    uart_queue_.ResetUartChunks();
}

bool UsbLink::ReceiveSensorReport(const bridge::UsbInPacket& packet,
                                  uint64_t arrival_us) {
    // 实际传输长度和会话已由 CompleteReceive 检查。
    if (!bridge::HasHeader(packet.data(), packet.size())) {
        return false;
    }
    switch (bridge::Kind(packet[9])) {
        case bridge::Kind::CanReceiveReport: {
            bridge::CanReceiveReport report{};
            if (!bridge::Decode(packet, report)) {
                return false;
            }
            ReceiveCanReport(report, arrival_us);
            break;
        }
        case bridge::Kind::UartReceiveReport: {
            bridge::UartReceiveReport report{};
            if (!bridge::Decode(packet, report) ||
                !ReceiveUartReport(report, arrival_us)) {
                return false;
            }
            break;
        }
        case bridge::Kind::ImuSampleReport: {
            bridge::ImuSampleReport report{};
            if (!bridge::Decode(packet, report) ||
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

void UsbLink::ReceiveCanReport(const bridge::CanReceiveReport& report,
                               uint64_t arrival_us) {
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

bool UsbLink::ReceiveUartReport(const bridge::UartReceiveReport& report,
                                uint64_t arrival_us) {
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
            sbus_decoder_.Discontinuity();
        }
        // 遥控丢帧计时继续使用主机实际接收时间。
        sbus_decoder_.Consume(bytes, arrival_us);
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

bool UsbLink::ReceiveOnboardImuReport(const bridge::ImuSampleReport& report,
                                      uint64_t arrival_us) {
    uint64_t sample_us = 0;
    if (!SampleTime(report.header.transmit_timestamp_us,
                    report.sample_timestamp_us, arrival_us, sample_us)) {
        return false;
    }
    onboard_imu_.Consume(report.spi_bytes, report.temperature_c,
                         report.sample_sequence, sample_us);
    return true;
}
}  // namespace wbr::host
