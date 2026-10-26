/* SPDX-License-Identifier: Apache-2.0 */
// 固定报文用例同时用于主机回归和 MCU 来源兼容性检查。
#ifndef DUST_USB_PROTOCOL_HEADER
#define DUST_USB_PROTOCOL_HEADER "hardware/usb/protocol/usb_protocol.hpp"
#endif
#include DUST_USB_PROTOCOL_HEADER
#ifdef DUST_USB_LEGACY_SESSION_HEADER
#include DUST_USB_LEGACY_SESSION_HEADER
#endif

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

namespace {
#ifndef DUST_USB_WIRE_NAMESPACE
#define DUST_USB_WIRE_NAMESPACE dust_control::wire
#endif
namespace wire = DUST_USB_WIRE_NAMESPACE;

template <typename Message>
void Emit(const std::string& name, const Message& message,
          wire::UsbOutRequest& packet) {
    if (!wire::Encode(message, packet)) {
        std::cerr << "Cannot encode " << name << '\n';
        std::exit(1);
    }
    std::cout << name << " 128 ";
    for (const auto byte : packet) {
        std::cout << std::hex << std::setfill('0') << std::setw(2)
                  << unsigned(byte);
    }
    std::cout << std::dec << '\n';
}

template <typename Message>
void Emit(const std::string& name, const Message& message,
          wire::UsbInPacket& packet) {
    if (!wire::Encode(message, packet)) {
        std::cerr << "Cannot encode " << name << '\n';
        std::exit(1);
    }
    std::cout << name << ' ' << wire::UsbReportLength(packet) << ' ';
    // 同时冻结实际长度和完整缓冲区，避免遗漏末尾填充变化。
    for (const auto byte : packet) {
        std::cout << std::hex << std::setfill('0') << std::setw(2)
                  << unsigned(byte);
    }
    std::cout << std::dec << '\n';
}
}  // namespace

int main() {
    const wire::UsbRequestHeader request{0x12345678, 0x90abcdef};
    const wire::UsbReportHeader report{0x87654321, 0x90abcdef};
    wire::UsbOutRequest out{};
    wire::UsbInPacket in{};
    Emit("hello", wire::HelloRequest{request}, out);
    Emit("arm", wire::ArmRequest{request}, out);
    Emit("stop", wire::StopRequest{request}, out);
    Emit("heartbeat", wire::HeartbeatRequest{request}, out);

    wire::CanTransmitRequest can{};
    can.header = request;
    wire::CanReceiveReport received{};
    received.header = report;
    for (unsigned i = 0; i < 6; ++i) {
        can.frames[i] = {uint8_t(i % 4),
                         uint8_t(i + 1),
                         uint8_t(i % 4),
                         i % 2 ? 0x1234567U : 0x345U,
                         {}};
        for (unsigned byte = 0; byte < 8; ++byte) {
            can.frames[i].data[byte] = uint8_t(0x80 + 8 * i + byte);
        }
        received.frames[i] = {can.frames[i], 0xfffffff0U + i, 0x10203040U + i};
    }
    for (unsigned count = 0; count <= 6; ++count) {
        can.frame_count = uint8_t(count);
        Emit("can_out_" + std::to_string(count), can, out);
        if (count) {
            received.frame_count = uint8_t(count);
            Emit("can_in_" + std::to_string(count), received, in);
        }
    }
    for (unsigned channel = 1; channel <= 6; ++channel) {
        wire::UartTransmitRequest uart{};
        uart.header = request;
        uart.channel = uint8_t(channel);
        uart.byte_count = channel == 1 ? 1 : 96;
        for (unsigned i = 0; i < uart.byte_count; ++i) {
            uart.bytes[i] = uint8_t(i + channel);
        }
        Emit("uart_out_" + std::to_string(channel), uart, out);
        wire::UartReceiveReport uart_in{};
        uart_in.header = report;
        uart_in.channel = uint8_t(channel);
        uart_in.byte_count = channel == 1 ? 1 : 40;
        uart_in.block_sequence = 0xfedcba98;
        uart_in.receive_timestamp_us = 0xfffffff0;
        for (unsigned i = 0; i < uart_in.byte_count; ++i) {
            uart_in.bytes[i] = uint8_t(0xff - i);
        }
        Emit("uart_in_" + std::to_string(channel), uart_in, in);
    }
    wire::ConfigureSafetyRequest safety{};
    safety.header = request;
    safety.lease_us = 20000;
    for (unsigned i = 0; i < 6; ++i) {
        auto& action = safety.uart_stop_actions[i];
        action.channel = uint8_t(i + 1);
        action.length = uint8_t(i + 1);
        for (unsigned byte = 0; byte < 8; ++byte) {
            action.data[byte] = uint8_t(0xf0 + byte);
        }
    }
    for (unsigned count = 0; count <= 6; ++count) {
        safety.uart_stop_count = uint8_t(count);
        Emit("safety_" + std::to_string(count), safety, out);
    }
    wire::ReadyReport ready{};
    ready.header = report;
    ready.capabilities = 255;
    ready.can_channel_mask = 15;
    ready.protection_frame_count = 6;
    ready.protection_signature = 0x12345678;
    ready.max_lease_us = 20000;
    Emit("ready", ready, in);
    wire::ImuSampleReport imu{};
    imu.header = report;
    imu.sample_timestamp_us = 0xfffffff0;
    imu.temperature_c = -12.5F;
    imu.sample_sequence = 0x12345678;
    for (unsigned i = 0; i < imu.spi_bytes.size(); ++i) {
        imu.spi_bytes[i] = uint8_t(0x70 + i);
    }
    Emit("imu", imu, in);
    for (unsigned armed = 0; armed <= 1; ++armed) {
        wire::RequestResultReport result{};
        result.header = report;
        result.request_sequence = request.request_sequence;
        result.request_kind = wire::Kind::Arm;
        result.armed = armed;
        result.result_code = armed ? 0 : -22;
        Emit("result_" + std::to_string(armed), result, in);
    }
    for (unsigned target = 0; target <= 2; ++target) {
        wire::StatusRequest status{};
        status.header = request;
        status.target = wire::StatusTarget(target);
        status.channel = 3;
        Emit("status_request_" + std::to_string(target), status, out);
    }
    wire::SessionStatusReport session{};
    session.header = report;
    session.channel = 3;
    session.armed = true;
    session.configured = true;
    session.stop_reason = wire::StopReason::Replaced;
    session.rejected_requests = 1;
    session.can_report_drops = 2;
    session.sensor_report_drops = 3;
    session.stop_errors = 4;
    session.reply_drops = 5;
    session.usb_send_errors = 6;
    session.usb_receive_drops = 7;
    session.usb_endpoint_errors = 8;
    session.protection_signature = 0x12345678;
    session.max_lease_us = 20000;
    Emit("status_session", session, in);
    wire::CanStatusReport can_status{report, 3, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    Emit("status_can", can_status, in);
    wire::UartStatusReport uart_status{report, 6, 1, 2, 3, 4};
    Emit("status_uart", uart_status, in);
    std::cout << "signature "
              << wire::ProtectionSignature(20000, can.frames.data(), 6) << '\n';
}
