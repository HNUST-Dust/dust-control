/* SPDX-License-Identifier: Apache-2.0 */
#include <algorithm>
#include <cassert>
#include <hardware/usb/protocol/usb_protocol.hpp>

#include "../support/usb_report_fixture.hpp"

int main() {
    namespace wire = dust::wire;
    wbr::host::UartReceiveQueue store;
    dust::test::UsbReportFixture decoder(store);
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
        wbr::host::UartReceiveChunk chunk{};
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
    wbr::host::UartReceiveChunk chunk{};
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
