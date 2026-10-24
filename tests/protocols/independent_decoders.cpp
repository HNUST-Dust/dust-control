/* SPDX-License-Identifier: Apache-2.0 */
#include <array>
#include <cassert>
#include <cstring>
#include <span>

#include "hardware/imu/hi91_decoder.hpp"
#include "hardware/imu/hi91_protocol.h"
#include "hardware/imu/onboard_imu.hpp"
#include "hardware/motors/motor_feedback.hpp"
#include "hardware/motors/motor_map.h"
#include "hardware/referee/referee_decoder.hpp"
#include "hardware/remote/sbus_decoder.hpp"

namespace {
void Put16(uint8_t* bytes, uint16_t value) {
    bytes[0] = uint8_t(value);
    bytes[1] = uint8_t(value >> 8);
}
std::array<uint8_t, 25> RemoteFrame() {
    std::array<uint8_t, 25> frame{};
    frame[0] = 15;
    frame[7] = 0x40;  // 通道 4 为中位，通道 5 为低位：允许遥控。
    return frame;
}
std::array<uint8_t, 82> ImuFrame() {
    std::array<uint8_t, 82> frame{};
    frame[0] = 0x5a;
    frame[1] = 0xa5;
    Put16(frame.data() + 2, 76);
    frame[6] = 0x91;
    frame[14] = 1;  // 设备时间 1 ms。
    const float pitch = 3.0F;
    std::memcpy(frame.data() + 58, &pitch, sizeof(pitch));
    Put16(frame.data() + 4,
          protocols::CalculateHi91FrameCrc(76, frame.data() + 6));
    return frame;
}
}  // namespace

int main() {
    using namespace dust::devices;
    const auto remote = RemoteFrame();
    for (size_t split = 0; split < remote.size(); ++split) {
        SbusDecoder decoder;
        decoder.Consume(std::span(remote).first(split), 1000);
        assert(!decoder.Read().remote.sequence);
        decoder.Consume(std::span(remote).subspan(split), 1000);
        assert(decoder.Read().remote.robot_enable);
    }
    SbusDecoder first;
    SbusDecoder second;
    first.Consume(remote, 1000);
    assert(first.Read().remote.robot_enable &&
           !second.Read().remote.robot_enable);
    first.Discontinuity();
    assert(!first.Read().remote.robot_enable);
    second.Consume(std::span(remote).first(10), 1000);
    second.Reset();
    second.Consume(std::span(remote).subspan(10), 1000);
    assert(!second.Read().remote.robot_enable);

    const auto imu = ImuFrame();
    for (size_t split = 0; split < imu.size(); ++split) {
        Hi91Decoder decoder(true);
        decoder.Consume(std::span(imu).first(split), 1000);
        assert(!decoder.Read().valid);
        decoder.Consume(std::span(imu).subspan(split), 1000);
        assert(decoder.Read().valid && decoder.Read().pitch_deg == 3.0F);
        assert(decoder.Read().precise_timestamp_us == 1000);
    }
    Hi91Decoder hi91(true);
    auto corrupted = imu;
    corrupted[4] ^= 1;
    hi91.Consume(corrupted, 1000);
    assert(!hi91.Read().valid);
    hi91.Consume(imu, 2000);
    assert(hi91.Read().valid && hi91.Read().sequence == 1);
    hi91.Consume(imu, 3000);
    assert(hi91.Read().sequence == 1 &&
           hi91.Read().precise_timestamp_us == 2000);
    hi91.Discontinuity();
    assert(!hi91.Read().valid);
    hi91.Reset();
    hi91.Consume(imu, 4000);
    assert(hi91.Read().valid && hi91.Read().sequence == 1);

    // 各 HI91 实例的恢复阈值独立，且不借用可被外部修改的配置。
    ImuConfig fast_config{1000};
    Hi91Decoder fast(true, fast_config);
    Hi91Decoder slow(true, ImuConfig{10000});
    fast_config.freshness_us = 10000;
    auto clock_frame = ImuFrame();
    auto set_device_time = [&](uint32_t device_ms) {
        for (unsigned byte = 0; byte < 4; ++byte) {
            clock_frame[14 + byte] =
                static_cast<uint8_t>(device_ms >> (8 * byte));
        }
        Put16(clock_frame.data() + 4,
              protocols::CalculateHi91FrameCrc(76, clock_frame.data() + 6));
    };
    set_device_time(1000);
    fast.Consume(clock_frame, 1000);
    slow.Consume(clock_frame, 1000);
    for (unsigned tick = 0; tick < 3; ++tick) {
        set_device_time(tick + 1);
        fast.Consume(clock_frame, 3000 + tick * 1000);
        slow.Consume(clock_frame, 3000 + tick * 1000);
    }
    assert(fast.Read().precise_timestamp_us == 5000);
    assert(slow.Read().precise_timestamp_us == 1000);

    MotorFeedbackDecoder motors;
    const auto& address = platform::motor_map::kDefaultMotors[0];
    const std::array<uint8_t, 8> raw{0, 0, 0, 123, 0, 0, 0, 0};
    motors.Consume(address.bus, address.feedback_id, raw, 1234, true);
    assert(motors.Read()[0].valid && motors.Read()[0].speed_rpm == 123);
    motors.Consume(address.bus, 0x7ff, raw, 2000, true);
    assert(motors.Read()[0].precise_timestamp_us == 1234);
    motors.Consume(address.bus, address.feedback_id, raw, 0, false);
    assert(!motors.Read()[0].valid);
    motors.Reset();
    assert(!motors.Read()[0].valid && motors.Read()[0].speed_rpm == 0);

    // 裁判协议沿用现有无 CRC 校验的边界解析，输出拥有完整载荷。
    const std::array<uint8_t, 12> referee{0xa5, 3,    0,  1, 0, 1,
                                          0,    0x21, 60, 0, 0, 0};
    RefereeDecoder referee_decoder;
    referee_decoder.Consume(std::span(referee).first(6), 1000);
    assert(!referee_decoder.Read().frame_sequence);
    referee_decoder.Consume(std::span(referee).subspan(6), 2000);
    assert(referee_decoder.Read().frame_sequence == 1);
    assert(referee_decoder.Read().command_id == 1 &&
           referee_decoder.Read().payload[0] == 0x21);
    assert(referee_decoder.Read().frame_us == 2000);
    referee_decoder.Reset();
    referee_decoder.Consume(std::span(referee).first(6), 3000);
    referee_decoder.Discontinuity();
    referee_decoder.Consume(std::span(referee).subspan(6), 4000);
    assert(!referee_decoder.Read().frame_sequence);
    referee_decoder.Consume(referee, 5000);
    assert(referee_decoder.Read().frame_sequence == 1);

    OnboardImu onboard;
    std::array<uint8_t, 25> onboard_raw{};
    onboard_raw[0] = 42;
    onboard.Consume(onboard_raw, 25.0F, 7, 1234);
    onboard_raw[0] = 0;
    assert(onboard.Read().raw[0] == 42 && onboard.Read().sample_us == 1234);
    assert(onboard.Read().temperature_c == 25.0F &&
           onboard.Read().sequence == 7);
    onboard.Reset();
    assert(onboard.Read().sequence == 0);
}
