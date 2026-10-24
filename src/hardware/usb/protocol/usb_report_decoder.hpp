/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "usb_protocol.hpp"

namespace dust::wire {
// MCU 只编码上行；主机补充以下解码接口，不修改三个共享文件。
// 调用方须先用实际传输长度核对 UsbReportLength；这里的数组长度只是容量。
// 解码失败不修改输出，成功后输出不引用输入缓冲区。
inline bool Decode(const UsbInPacket& packet, CanReceiveReport& message);
inline bool Decode(const UsbInPacket& packet, UartReceiveReport& message);
inline bool Decode(const UsbInPacket& packet, ImuSampleReport& message);
inline bool Decode(const UsbInPacket& packet, ReadyReport& message);
inline bool Decode(const UsbInPacket& packet, RequestResultReport& message);

namespace report_detail {
inline bool HasReportKind(const UsbInPacket& packet, Kind kind) {
    return HasHeader(packet.data(), packet.size()) && Kind(packet[9]) == kind;
}
inline UsbReportHeader DecodeHeader(const UsbInPacket& packet) {
    return {U32(packet.data() + 4), U32(packet.data() + 12)};
}
}  // namespace report_detail

inline bool Decode(const UsbInPacket& packet, CanReceiveReport& message) {
    if (!report_detail::HasReportKind(packet, Kind::CanReceiveReport) ||
        !packet[10] || packet[10] > kMaxCanFramesPerPacket) {
        return false;
    }
    CanReceiveReport result{};
    result.header = report_detail::DecodeHeader(packet);
    result.frame_count = packet[10];
    for (size_t i = 0; i < result.frame_count; ++i) {
        const auto* record = packet.data() + 16 + i * kCanReceiveRecordBytes;
        if (!ValidRecord(record)) {
            return false;
        }
        result.frames[i] = {Decode(record), U32(record + 16), U32(record + 20)};
    }
    message = result;
    return true;
}

inline bool Decode(const UsbInPacket& packet, UartReceiveReport& message) {
    if (!report_detail::HasReportKind(packet, Kind::UartReceiveReport) ||
        packet[10] < 1 || packet[10] > kUartPortCount || !packet[11] ||
        packet[11] > 40) {
        return false;
    }
    UartReceiveReport result{};
    result.header = report_detail::DecodeHeader(packet);
    result.channel = packet[10];
    result.byte_count = packet[11];
    result.block_sequence = U32(packet.data() + 16);
    result.receive_timestamp_us = U32(packet.data() + 20);
    std::memcpy(result.bytes.data(), packet.data() + 24, result.byte_count);
    message = result;
    return true;
}

inline bool Decode(const UsbInPacket& packet, ImuSampleReport& message) {
    if (!report_detail::HasReportKind(packet, Kind::ImuSampleReport) ||
        packet[10] != 25) {
        return false;
    }
    ImuSampleReport result{};
    result.header = report_detail::DecodeHeader(packet);
    result.sample_timestamp_us = U32(packet.data() + 16);
    std::memcpy(result.spi_bytes.data(), packet.data() + 20, 25);
    const auto temperature_bits = U32(packet.data() + 48);
    std::memcpy(&result.temperature_c, &temperature_bits, 4);
    result.sample_sequence = U32(packet.data() + 52);
    message = result;
    return true;
}

inline bool Decode(const UsbInPacket& packet, ReadyReport& message) {
    if (!report_detail::HasReportKind(packet, Kind::Ready)) {
        return false;
    }
    ReadyReport result{};
    result.header = report_detail::DecodeHeader(packet);
    result.capabilities = U32(packet.data() + 16);
    result.request_bytes = U32(packet.data() + 20);
    result.max_report_bytes = U32(packet.data() + 24);
    result.can_channel_mask = U32(packet.data() + 28);
    result.max_can_frames = packet[32];
    result.protection_frame_count = packet[33];
    result.protection_flags = packet[34];
    result.protection_signature = U32(packet.data() + 36);
    result.max_lease_us = U32(packet.data() + 40);
    message = result;
    return true;
}

inline bool Decode(const UsbInPacket& packet, RequestResultReport& message) {
    if (!report_detail::HasReportKind(packet, Kind::Result) || packet[21] > 1) {
        return false;
    }
    RequestResultReport result{};
    result.header = report_detail::DecodeHeader(packet);
    result.request_sequence = U32(packet.data() + 16);
    result.request_kind = Kind(packet[20]);
    result.armed = packet[21];
    result.result_code = int32_t(U32(packet.data() + 24));
    message = result;
    return true;
}
}  // namespace dust::wire
