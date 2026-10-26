/* SPDX-License-Identifier: Apache-2.0 */
#include <algorithm>
#include <cassert>
#include <cstring>
#include <hardware/usb/protocol/usb_protocol.hpp>

#include "../support/usb_report_fixture.hpp"
#include "hardware/imu/hi91_protocol.h"

void TestBindings(dust_control::host::UartSensorBindings bindings) {
    namespace wire = dust_control::wire;
    dust_control::host::UartReceiveQueue store;
    dust_control::host::UsbOptions options;
    options.uart_sensors = bindings;
    dust_control::test::UsbReportFixture decoder(store, options);
    std::array<uint32_t, 7> sequences{};
    auto send = [&](uint8_t channel, std::span<const uint8_t> bytes) {
        while (!bytes.empty()) {
            wire::UartReceiveReport report{};
            report.header = {1000, 1};
            report.channel = channel;
            report.block_sequence = sequences[channel]++;
            report.receive_timestamp_us = 900;
            report.byte_count = std::min<size_t>(32, bytes.size());
            std::copy_n(bytes.begin(), report.byte_count, report.bytes.begin());
            bytes = bytes.subspan(report.byte_count);
            wire::UsbInPacket packet{};
            assert(wire::Encode(report, packet));
            assert(decoder.Consume(packet, 2000));
        }
    };
    std::array<uint8_t, 25> remote{};
    remote[0] = 15;
    remote[7] = 0x40;
    std::array<uint8_t, 82> imu{};
    imu[0] = 0x5a;
    imu[1] = 0xa5;
    imu[2] = 76;
    imu[6] = 0x91;
    imu[14] = 1;
    const float pitch = 3.0F;
    std::memcpy(imu.data() + 58, &pitch, sizeof(pitch));
    const auto crc =
        dust_control::protocols::CalculateHi91FrameCrc(76, imu.data() + 6);
    imu[4] = uint8_t(crc);
    imu[5] = uint8_t(crc >> 8);
    const std::array<uint8_t, 12> referee{0xa5, 3,    0,  1, 0, 1,
                                          0,    0x21, 60, 0, 0, 0};
    // 每个物理通道都送入有效帧：只有绑定的设备才应更新。
    for (uint8_t channel = 1; channel <= 6; ++channel) {
        decoder.Reset();
        send(channel, remote);
        assert(bool(decoder.Read().control.remote.sequence) ==
               (channel == bindings.remote));
        decoder.Reset();
        send(channel, imu);
        assert(decoder.Read().control.imu.valid == (channel == bindings.hi91));
        decoder.Reset();
        send(channel, referee);
        assert(bool(decoder.Read().referee.frame_sequence) ==
               (channel == bindings.referee));
        dust_control::host::UartReceiveChunk raw{};
        assert(store.TakeUartChunk(channel, raw));
    }
    // 丢块必须使已发布的遥控数据失效。
    if (bindings.remote) {
        decoder.Reset();
        send(bindings.remote, remote);
        assert(decoder.Read().control.remote.robot_enable);
        ++sequences[bindings.remote];
        send(bindings.remote, std::span(remote).first(1));
        assert(!decoder.Read().control.remote.robot_enable);
    }
}

int main() {
    TestBindings({1, 2, 3});
    TestBindings({4, 6, 5});
    TestBindings({0, 0, 0});
    for (auto invalid : {dust_control::host::UartSensorBindings{1, 1, 3},
                         dust_control::host::UartSensorBindings{7, 2, 3}}) {
        bool rejected = false;
        try {
            invalid.Validate();
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        assert(rejected);
    }

    namespace wire = dust_control::wire;
    dust_control::host::UartReceiveQueue store;
    dust_control::test::UsbReportFixture decoder(store);
    wire::UsbInPacket packet{};
    wire::Header(packet.data(), wire::Kind::UartReceiveReport, 1000, 1);
    packet[11] = 3;
    packet[24] = 0xa5;
    packet[25] = 0x5a;
    packet[26] = 0x01;
    wire::Put32(packet.data() + 20, 1000);
    for (uint8_t channel = 1; channel <= wire::kUartPortCount; ++channel) {
        wire::UartTransmitRequest message{};
        message.header = {1, 1};
        message.channel = channel;
        message.byte_count = 3;
        std::copy_n(packet.data() + 24, 3, message.bytes.begin());
        wire::UsbOutRequest request{};
        assert(wire::Encode(message, request));
        wire::UartTransmitRequest decoded{};
        assert(wire::Decode(request, decoded));
        assert(decoded.channel == channel && decoded.byte_count == 3);
        packet[10] = channel;
        wire::Put32(packet.data() + 16, 7);
        assert(decoder.Consume(packet, 2000));
        dust_control::host::UartReceiveChunk chunk{};
        assert(store.TakeUartChunk(channel, chunk));
        assert(chunk.sequence == 7 && chunk.size == 3 &&
               chunk.bytes[1] == 0x5a);
        assert(chunk.receive_us == 2000 && !chunk.discontinuity);
        wire::Put32(packet.data() + 16, 9);
        assert(decoder.Consume(packet, 2001));
        assert(store.TakeUartChunk(channel, chunk) && chunk.discontinuity);
    }
    packet[10] = 7;
    assert(!decoder.Consume(packet, 2002));
    packet[10] = 0;
    assert(!decoder.Consume(packet, 2002));
    packet[10] = 6;
    packet[11] = 0;
    assert(!decoder.Consume(packet, 2002));
    packet[11] = 41;
    assert(!decoder.Consume(packet, 2002));
    packet[11] = 3;
    wire::Put32(packet.data() + 20, 1000);
    for (unsigned sequence = 10; sequence < 43; ++sequence) {
        wire::Put32(packet.data() + 16, sequence);
        assert(decoder.Consume(packet, 2003));
    }
    assert(store.UartDroppedChunks(6) == 1);
    dust_control::host::UartReceiveChunk chunk{};
    assert(store.TakeUartChunk(6, chunk) && chunk.sequence == 11 &&
           chunk.discontinuity);
    decoder.Reset();
    assert(!store.TakeUartChunk(6, chunk));
    assert(store.UartDroppedChunks(6) == 0);
    assert(!store.TakeUartChunk(7, chunk));

    // UART 3 由裁判解析器跨 USB 块拼帧，仍保留原始块队列。
    const std::array<uint8_t, 12> referee{0xa5, 3,    0,  1, 0, 1,
                                          0,    0x21, 60, 0, 0, 0};
    wire::UartReceiveReport referee_report{};
    referee_report.header = {1000, 1};
    referee_report.channel = 3;
    referee_report.byte_count = 6;
    referee_report.receive_timestamp_us = 900;
    std::copy_n(referee.begin(), 6, referee_report.bytes.begin());
    assert(wire::Encode(referee_report, packet));
    assert(decoder.Consume(packet, 2000));
    assert(!decoder.Read().referee.frame_sequence);
    referee_report.block_sequence = 1;
    std::copy_n(referee.begin() + 6, 6, referee_report.bytes.begin());
    assert(wire::Encode(referee_report, packet));
    assert(decoder.Consume(packet, 3000));
    const auto parsed = decoder.Read().referee;
    assert(parsed.command_id == 1 && parsed.payload[0] == 0x21);
    assert(parsed.frame_sequence == 1 && parsed.frame_us == 2900);
    assert(parsed.chunk_size == 6 && parsed.receive_us == 2900);

    // 板载 IMU 只更新自己的原始样本，不覆盖 HI91 控制源。
    wire::ImuSampleReport onboard{};
    onboard.header = {1000, 1};
    onboard.sample_timestamp_us = 950;
    onboard.spi_bytes[0] = 42;
    onboard.temperature_c = 25.0F;
    onboard.sample_sequence = 7;
    assert(wire::Encode(onboard, packet));
    assert(decoder.Consume(packet, 4000));
    const auto snapshot = decoder.Read();
    assert(snapshot.onboard.raw[0] == 42);
    assert(snapshot.onboard.sample_us == 3950);
    assert(snapshot.onboard.temperature_c == 25.0F);
    assert(snapshot.onboard.sequence == 7);
    assert(!snapshot.control.imu.valid);
    decoder.Reset();
    assert(!decoder.Read().referee.frame_sequence);
    assert(!decoder.Read().onboard.sequence);
}
