#include "hardware/usb/protocol/usb_protocol.hpp"
/* SPDX-License-Identifier: Apache-2.0 */
#include <cstdlib>
#include <iostream>
#define CHECK(c)                                      \
    do {                                              \
        if (!(c)) {                                   \
            std::cerr << "line " << __LINE__ << '\n'; \
            std::abort();                             \
        }                                             \
    } while (0)
int main() {
    dust_control::wire::UsbInPacket can_report{};
    can_report[9] = uint8_t(dust_control::wire::Kind::CanReceiveReport);
    for (unsigned count = 1; count <= 6; ++count) {
        can_report[10] = count;
        CHECK(dust_control::wire::UsbReportLength(can_report) ==
              16 + 24 * count);
    }
    can_report[10] = 7;
    CHECK(dust_control::wire::UsbReportLength(can_report) == 0);

    namespace w = dust_control::wire;
    w::SessionPolicy policy;
    CHECK(!policy.Arm(1, 0));
    policy.Open(42);
    CHECK(!policy.Arm(1, 0));
    CHECK(!policy.Configure(0));
    CHECK(!policy.Configure(1000001));
    CHECK(policy.Configure(20000));
    policy.Observe(1);
    CHECK(!policy.Arm(1, 0));
    CHECK(policy.Arm(2, 0xfffffff0U));
    CHECK(!policy.Configure(10000));
    CHECK(!policy.Accept(2, 100));  // 重放不能续租。
    CHECK(!policy.Accept(1, 100));
    CHECK(!policy.Expired(uint32_t(0xfffffff0U + 19999)));
    CHECK(policy.Expired(uint32_t(0xfffffff0U + 20000)));
    CHECK(!policy.Accept(3, uint32_t(0xfffffff0U + 20000)));
    policy.Close();
    policy.Open(43);
    CHECK(policy.Configure(20000));
    CHECK(policy.Arm(3, 0));
    CHECK(policy.Expired(20000));
    CHECK(!policy.Accept(4, 20001));  // 超时后必须建立新会话。
    policy.Close();
    CHECK(!policy.Armed() && !policy.Configured() && !policy.Session());
    CHECK(!policy.Accept(4, 0));
    CHECK(w::NewSequence(0, 0xffffffffU));
    w::CanFrame frame{};
    frame.channel = 0;
    frame.id = 0x7ff;
    frame.length = 8;
    frame.data[7] = 0x81;
    w::CanTransmitRequest message{};
    message.header = {1, 42};
    message.frame_count = 1;
    message.frames[0] = frame;
    w::UsbOutRequest request{};
    CHECK(w::Encode(message, request));
    w::CanTransmitRequest received{};
    CHECK(w::Decode(request, received));
    CHECK(received.header.session_id == 42 && received.frame_count == 1);
    CHECK(request[9] == 3 && request[10] == 1 && request[33] == 8);
    CHECK(w::U32(request.data() + 36) == 0x7ff && request[47] == 0x81);
    CHECK(w::HasHeader(request.data(), 128));
    CHECK(w::ValidRecords(request));
    CHECK(w::Decode(request.data() + 32).data[7] == 0x81);
    request[32 + 3] = 1;
    CHECK(!w::ValidRecords(request));
    request[32 + 3] = 0;
    request[10] = 7;
    CHECK(!w::ValidRecords(request));
    frame.id = 0x1fffffff;
    frame.flags = w::kExtended | w::kRemote;
    message.header.request_sequence = 2;
    message.frames[0] = frame;
    CHECK(w::Encode(message, request));
    CHECK(w::ValidRecords(request));
    request[32 + 2] = 0;
    CHECK(!w::ValidRecords(request));
    request = {};
    request[8] = 1;
    CHECK(!w::HasHeader(request.data(), 128));
    w::ConfigureSafetyRequest safety{};
    safety.header = {3, 42};
    safety.lease_us = 20000;
    safety.uart_stop_count = 1;
    safety.uart_stop_actions[0].channel = 2;
    safety.uart_stop_actions[0].length = 1;
    safety.uart_stop_actions[0].data[0] = 0xfd;
    CHECK(w::Encode(safety, request));
    w::ConfigureSafetyRequest safety_decoded{};
    CHECK(w::Decode(request, safety_decoded));
    CHECK(safety_decoded.uart_stop_count == 1 &&
          safety_decoded.uart_stop_actions[0].data[0] == 0xfd);
    CHECK(!w::ValidRecords(request));
    request[20] = 1;
    CHECK(!w::Decode(request, safety_decoded));
    w::UartTransmitRequest uart{};
    uart.header = {4, 42};
    uart.channel = 3;
    uart.byte_count = 96;
    CHECK(w::Encode(uart, request));
    CHECK(request[11] == 96);
    w::UartTransmitRequest uart_decoded{};
    CHECK(w::Decode(request, uart_decoded));
    uart.byte_count = 97;
    CHECK(!w::Encode(uart, request));

    // 使用 MCU 编码器生成上行，验证主机解码和固定 v4 字段位置。
    w::CanReceiveReport report{};
    report.header = {0x12345678, 42};
    for (unsigned count = 1; count <= 6; ++count) {
        report.frame_count = count;
        for (unsigned i = 0; i < count; ++i) {
            report.frames[i].frame = {3, 8, 0, 0x201, {}};
            report.frames[i].frame.data[7] = uint8_t(i);
            report.frames[i].receive_timestamp_us = 0x12345670 + i;
            report.frames[i].bus_sequence = 100 + i;
        }
        CHECK(w::Encode(report, can_report));
        CHECK(w::UsbReportLength(can_report) == 16 + 24 * count);
        w::CanReceiveReport decoded{};
        CHECK(w::Decode(can_report, decoded));
        CHECK(decoded.frame_count == count && decoded.header.session_id == 42);
        CHECK(decoded.frames[count - 1].bus_sequence == 99 + count);
        CHECK(w::U32(can_report.data() + 20) == 0x201);
        can_report[16 + 24 * (count - 1) + 3] = 1;
        decoded.header.session_id = 99;
        CHECK(!w::Decode(can_report, decoded));
        CHECK(decoded.header.session_id == 99);  // 失败不提交部分解码结果。
    }
    w::ReadyReport ready{};
    ready.header = {1, 42};
    ready.capabilities = 255;
    ready.can_channel_mask = 12;
    ready.max_lease_us = 20000;
    CHECK(w::Encode(ready, can_report));
    w::ReadyReport ready_decoded{};
    CHECK(w::Decode(can_report, ready_decoded));
    CHECK(ready_decoded.capabilities == 255 &&
          ready_decoded.max_lease_us == 20000);
    CHECK(w::U32(can_report.data() + 24) == 160);
    w::RequestResultReport result{};
    result.header = {1, 42};
    result.request_sequence = 7;
    result.request_kind = w::Kind::Arm;
    result.armed = true;
    result.result_code = -22;
    CHECK(w::Encode(result, can_report));
    w::RequestResultReport result_decoded{};
    CHECK(w::Decode(can_report, result_decoded));
    CHECK(result_decoded.result_code == -22 && result_decoded.armed);
    can_report[21] = 2;
    CHECK(!w::Decode(can_report, result_decoded));
    w::ImuSampleReport imu{};
    imu.header = {10, 42};
    imu.sample_timestamp_us = 8;
    imu.temperature_c = 37.5F;
    imu.spi_bytes[24] = 0xa5;
    CHECK(w::Encode(imu, can_report));
    w::ImuSampleReport imu_decoded{};
    CHECK(w::Decode(can_report, imu_decoded));
    CHECK(imu_decoded.temperature_c == 37.5F &&
          imu_decoded.spi_bytes[24] == 0xa5);
    CHECK(can_report[10] == 25 && can_report[44] == 0xa5);
    can_report[10] = 24;
    CHECK(!w::Decode(can_report, imu_decoded));
    std::cout
        << "Generic protocol bounds, flags, session gating and lease wrap "
           "contracts passed\n";
}

// Wire v4：固定检查编号和版本，避免两端仅改名而未同步编码。
static_assert(dust_control::wire::kVersion == 4);
static_assert(uint8_t(dust_control::wire::Kind::CanTransmitRequest) == 3);
static_assert(uint8_t(dust_control::wire::Kind::CanReceiveReport) == 4);
static_assert(uint8_t(dust_control::wire::Kind::UartTransmitRequest) == 5);
static_assert(uint8_t(dust_control::wire::Kind::UartReceiveReport) == 6);
static_assert(uint8_t(dust_control::wire::Kind::ImuSampleReport) == 7);
static_assert(uint8_t(dust_control::wire::Kind::Stop) == 10);
static_assert(uint8_t(dust_control::wire::Kind::Heartbeat) == 11);
static_assert(uint8_t(dust_control::wire::Kind::Result) == 12);
static_assert(uint8_t(dust_control::wire::Kind::Status) == 13);
