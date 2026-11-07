/* SPDX-License-Identifier: Apache-2.0 */
#include <array>
#include <cassert>
#include <cstring>
#include <span>

#include "../support/motor_fixture.hpp"
#include "hardware/imu/hi91_imu.hpp"
#include "hardware/imu/onboard_imu.hpp"
#include "hardware/motors/motor_protocol.hpp"
#include "hardware/remote/wfly_sbus_protocol.h"

namespace {
std::array<uint8_t, 25> RemoteFrame() {
    std::array<uint8_t, 25> frame{};
    frame[0] = 15;
    frame[7] = 0x40;  // 通道 4 为中位，通道 5 为低位：允许遥控。
    return frame;
}
}  // namespace

int main() {
    using namespace dust_control::devices;
    const auto remote = RemoteFrame();
    for (size_t split = 0; split < remote.size(); ++split) {
        WflySbusDecoder decoder;
        decoder.Consume(std::span(remote).first(split), 1000);
        assert(!decoder.Read().remote.sequence);
        decoder.Consume(std::span(remote).subspan(split), 1000);
        assert(decoder.Read().remote.robot_enable);
    }
    WflySbusDecoder first;
    WflySbusDecoder second;
    first.Consume(remote, 1000);
    assert(first.Read().remote.robot_enable &&
           !second.Read().remote.robot_enable);
    first.Discontinuity();
    assert(!first.Read().remote.robot_enable);
    second.Consume(std::span(remote).first(10), 1000);
    second.Reset();
    second.Consume(std::span(remote).subspan(10), 1000);
    assert(!second.Read().remote.robot_enable);

    MotorFeedbackDecoder motors(dust_control::test::MotorFixture());
    const auto& address = dust_control::test::kMotorAddresses[0];
    const std::array<uint8_t, 8> raw{0, 0, 0, 123, 0, 0, 0, 0};
    motors.Consume(address.bus, address.feedback_id, raw, 1234, true);
    assert(motors.Read()[0].valid && motors.Read()[0].speed_rpm == 123);
    motors.Consume(address.bus, 0x7ff, raw, 2000, true);
    assert(motors.Read()[0].precise_timestamp_us == 1234);
    motors.Consume(address.bus, address.feedback_id, raw, 0, false);
    assert(!motors.Read()[0].valid);
    motors.Reset();
    assert(!motors.Read()[0].valid && motors.Read()[0].speed_rpm == 0);

    OnboardImu onboard;
    std::array<int16_t, 3> accel{4096, -4096, 0};
    const std::array<int16_t, 3> gyro{16384, -16384, 0};
    onboard.Consume(accel, gyro, 25.0F, 7, 1234);
    accel[0] = 0;
    assert(onboard.Read().accel_raw[0] == 4096 &&
           onboard.Read().sample_us == 1234);
    assert(onboard.Read().temperature_c == 25.0F &&
           onboard.Read().sequence == 7);
    assert(onboard.Read().accel_raw[0] == 4096 &&
           onboard.Read().accel_raw[1] == -4096);
    assert(onboard.Read().gyro_raw[0] == 16384 &&
           onboard.Read().gyro_raw[1] == -16384);
    assert(std::abs(onboard.Read().accel_mps2[0] - 9.80665F) < 0.0001F);
    assert(std::abs(onboard.Read().gyro_rad_s[0] - 17.453293F) < 0.0001F);
    onboard.Reset();
    assert(onboard.Read().sequence == 0);
}
