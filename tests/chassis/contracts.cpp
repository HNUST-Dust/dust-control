#include "chassis/chassis_input_reader.h"
#include "hardware/usb/protocol/usb_protocol.hpp"
#include "hardware/usb/usb_transmitter.hpp"
/* SPDX-License-Identifier: Apache-2.0 */
#include <hardware/imu/hi91_protocol.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "../support/motor_fixture.hpp"
#include "../support/usb_report_fixture.hpp"
#include "chassis_config.h"
#include "chassis_controller.h"
#include "chassis_input_reader.h"
#include "hardware/motors/motor_protocol.hpp"
#define CHECK(condition)                              \
    do {                                              \
        if (!(condition)) {                           \
            std::cerr << "line " << __LINE__ << '\n'; \
            return 1;                                 \
        }                                             \
    } while (false)
using namespace dust_control;
namespace {
dust_control::wire::UsbInPacket Uart(unsigned port, uint32_t sequence,
                                     const uint8_t* data, size_t size,
                                     uint32_t time = 1000) {
    dust_control::wire::UsbInPacket packet{};
    dust_control::wire::Header(
        packet.data(), dust_control::wire::Kind::UartReceiveReport, time, 7);
    packet[10] = port;
    packet[11] = size;
    dust_control::wire::Put32(packet.data() + 16, sequence);
    dust_control::wire::Put32(packet.data() + 20, time);
    std::copy_n(data, size, packet.data() + 24);
    return packet;
}

std::array<uint8_t, 25> Sbus(bool enabled) {
    std::array<uint8_t, 25> bytes{};
    bytes[0] = 15;
    for (unsigned channel = 0; channel < 16; ++channel) {
        uint16_t value = 1024;
        if (channel == 4) {
            value = enabled ? 1024 : 300;
        }
        if (channel == 5) {
            value = 300;
        }
        for (unsigned bit = 0; bit < 11; ++bit) {
            if (value & (1U << bit)) {
                bytes[1 + (channel * 11 + bit) / 8] |=
                    uint8_t(1U << ((channel * 11 + bit) % 8));
            }
        }
    }

    return bytes;
}
}  // namespace
int main() {
    {
        host::UartReceiveQueue remote_store;
        dust_control::test::UsbReportFixture remote_decoder(remote_store);
        uint32_t chunk_sequence = 0;
        auto send_remote = [&](std::array<uint8_t, 25> frame,
                               uint64_t host_us) {
            return remote_decoder.Consume(
                Uart(1, chunk_sequence++, frame.data(), 25), host_us);
        };
        auto enabled = Sbus(true);
        auto lost = enabled;
        lost[23] = 4;
        CHECK(send_remote(enabled, 10000));
        const auto sequence = remote_decoder.Read().control.remote.sequence;
        CHECK(send_remote(lost, 24000));
        CHECK(remote_decoder.Read().control.remote.robot_enable);
        CHECK(remote_decoder.Read().control.remote.sequence == sequence);
        CHECK(send_remote(enabled, 38000));  // 孤立丢帧不会禁用。
        CHECK(remote_decoder.Read().control.remote.robot_enable);
        CHECK(!remote_decoder.remote_timeout_pending());
        remote_decoder.PollRemoteTimeout(2000000);
        CHECK(
            remote_decoder.Read().control.remote.robot_enable);  // 计时已取消。
        CHECK(send_remote(lost, 2100000));
        const auto held_sequence =
            remote_decoder.Read().control.remote.sequence;
        CHECK(send_remote(lost,
                          2500000));  // 连续丢帧不能刷新状态。
        CHECK(remote_decoder.Read().control.remote.sequence == held_sequence);
        remote_decoder.PollRemoteTimeout(3099999);
        CHECK(remote_decoder.Read().control.remote.robot_enable);
        remote_decoder.PollRemoteTimeout(3100000);
        CHECK(!remote_decoder.Read().control.remote.robot_enable);
        CHECK(remote_decoder.Read().control.remote.sequence ==
              held_sequence + 1);
        remote_decoder.PollRemoteTimeout(4000000);
        CHECK(send_remote(lost, 4100000));
        CHECK(remote_decoder.Read().control.remote.sequence ==
              held_sequence + 1);
        CHECK(send_remote(enabled, 4200000));
        CHECK(remote_decoder.Read().control.remote.robot_enable);
        auto failsafe = enabled;
        failsafe[23] = 8;
        CHECK(send_remote(failsafe, 4201000));
        CHECK(
            !remote_decoder.Read().control.remote.robot_enable);  // 立即失效。
        CHECK(send_remote(enabled, 4202000));
        auto both = enabled;
        both[23] = 12;
        CHECK(send_remote(both, 4203000));
        CHECK(remote_decoder.Read()
                  .control.remote.robot_enable);  // 与 MCU 优先级完全一致。
        remote_decoder.Reset();
        remote_decoder.PollRemoteTimeout(6000000);
        CHECK(!remote_decoder.Read().control.remote.sequence);
        CHECK(!remote_decoder.remote_timeout_pending());
    }

    dust_control::devices::Hi91TimeGuard time_guard;
    CHECK(time_guard.Accept(100000, 1000, 3000, true));
    CHECK(!time_guard.Accept(10, 2000, 3000, true));  // 样本仍新鲜。
    CHECK(!time_guard.Accept(10, 10000, 3000, true));
    CHECK(!time_guard.Accept(11, 11000, 3000, true));
    CHECK(time_guard.Accept(12, 12000, 3000, true));
    CHECK(!time_guard.Accept(12, 13000, 3000, true));
    CHECK(time_guard.Accept(13, 14000, 3000, true));
    CHECK(!time_guard.Accept(1, 20000, 3000, false));  // 拒绝损坏的重启候选帧。
    CHECK(!time_guard.Accept(2, 21000, 3000, true));
    CHECK(!time_guard.Accept(1, 22000, 3000,
                             true));  // 乱序后重新确认恢复。
    CHECK(!time_guard.Accept(2, 23000, 3000, true));
    CHECK(time_guard.Accept(3, 24000, 3000, true));
    time_guard.Reset();
    CHECK(time_guard.Accept(0xfffffffeU, 1000, 3000, true));
    CHECK(time_guard.Accept(1, 2000, 3000, true));  // 正常 uint32 环绕。
    time_guard.Reset();
    CHECK(time_guard.Accept(100000, 1000, 3000, true));
    CHECK(!time_guard.Accept(1, 10000, 3000, true));
    CHECK(!time_guard.Accept(2, 10000, 3000, true));
    CHECK(!time_guard.Accept(3, 10000, 3000,
                             true));  // 单次缓冲突发不足以确认恢复。
    time_guard.Reject();
    CHECK(!time_guard.Accept(4, 11000, 3000, true));
    CHECK(!time_guard.Accept(5, 12000, 3000, true));
    CHECK(time_guard.Accept(6, 13000, 3000, true));
    dust_control::wire::SessionPolicy lease;
    lease.Open(1);
    CHECK(!lease.Arm(1, 0));
    CHECK(lease.Configure(dust_control::host::kCommandTimeoutUs));
    CHECK(lease.Arm(0xffffffffU, 0xfffffff0U));
    CHECK(!lease.Accept(0xffffffffU, 0));
    CHECK(lease.Fresh(0));
    CHECK(!lease.Expired(0x10U));
    CHECK(lease.Expired(
        uint32_t(0xfffffff0U + dust_control::host::kCommandTimeoutUs)));
    CHECK(lease.Accept(0, 0));
    CHECK(!lease.Expired(dust_control::host::kCommandTimeoutUs - 1));
    CHECK(lease.Expired(dust_control::host::kCommandTimeoutUs));
    lease.Close();
    CHECK(!lease.Armed());

    CHECK(dust_control::wire::NewSequence(0, 0xffffffffU));
    CHECK(!dust_control::wire::NewSequence(9, 10));
    dust_control::wire::CanTransmitRequest batch{};
    batch.frame_count = 1;
    batch.frames[0] = {3, 8, 0, 0x200, {}};
    batch.header = {1, 7};
    dust_control::wire::UsbOutRequest packet{};
    CHECK(dust_control::wire::Encode(batch, packet));
    dust_control::wire::CanTransmitRequest decoded{};
    CHECK(dust_control::wire::Decode(packet, decoded) &&
          decoded.header.session_id == 7);
    CHECK(decoded.header.session_id != 8);
    batch.frame_count = 2;
    batch.frames[1] = batch.frames[0];
    CHECK(
        dust_control::wire::Encode(batch, packet));  // 原始 FIFO 允许重复 ID。
    batch.frame_count = 7;
    CHECK(!dust_control::wire::Encode(batch, packet));
    dust_control::devices::MotorEncoder encoder(
        dust_control::test::MotorFixture());
    dust_control::chassis::ChassisController controller;
    dust_control::chassis::ControllerInputs sources{};
    sources.remote.robot_enable = true;
    sources.remote.sequence = 1;
    const auto disabled_output = controller.Tick(sources, 1000, false);
    CHECK(controller.state() ==
          dust_control::chassis::ChassisControlState::kDisabled);
    const auto encoded = encoder.Encode(disabled_output.motor_targets);
    CHECK(encoded.has_value());
    const auto& disabled = *encoded;
    CHECK(disabled.count == 6);
    const unsigned expected_slots[] = {0, 2, 3, 1, 4, 5};
    for (unsigned i = 0; i < 6; ++i) {
        const auto& address =
            dust_control::test::kMotorAddresses[expected_slots[i]];
        CHECK(disabled.frames[i].bus == address.bus);
        CHECK(disabled.frames[i].id == address.command_id);
        const auto& record = disabled.frames[i];
        if (record.id == 0x200) {
            for (unsigned j = 0; j < 8; ++j) {
                CHECK(record.bytes[j] == 0);
            }
        } else {
            CHECK(record.bytes[7] ==
                  0xfd);  // 仅退出命令，不能使能或产生非零力矩。
        }
    }

    dust_control::chassis::ChassisStateMachine states;
    dust_control::chassis::ChassisCycleInput input{};
    input.dt = .001;
    input.requested_enable = true;
    input.remote_fresh = true;
    input.imu_fresh = true;
    CHECK(states.Update(input).to ==
          dust_control::chassis::ChassisControlState::kDmArming);
    input.imu_fresh = false;
    CHECK(states.Update(input).to ==
          dust_control::chassis::ChassisControlState::kSafetyStop);
    input.requested_enable = false;
    CHECK(states.Update(input).to ==
          dust_control::chassis::ChassisControlState::kDisabled);
    host::UartReceiveQueue store;
    dust_control::test::UsbReportFixture decoder(store, true);
    auto sbus = Sbus(true);
    CHECK(decoder.Consume(Uart(1, 0, sbus.data(), 10), 2000));
    CHECK(!decoder.Read().control.remote.robot_enable);
    CHECK(decoder.Consume(Uart(1, 2, sbus.data() + 10, 15),
                          2000));  // 流间断后拒绝不完整帧。
    CHECK(!decoder.Read().control.remote.robot_enable);
    CHECK(decoder.Consume(Uart(1, 3, sbus.data(), 25), 2000));
    CHECK(decoder.Read().control.remote.robot_enable);
    auto safe = Sbus(false);
    CHECK(decoder.Consume(Uart(1, 4, safe.data(), 25), 2000));
    CHECK(decoder.Read().control.remote_disable_pose);
    safe[23] = 8;
    CHECK(decoder.Consume(Uart(1, 5, safe.data(), 25), 2000));
    CHECK(!decoder.Read().control.remote_disable_pose &&
          !decoder.Read().control.remote.robot_enable);
    safe[23] = 4;
    CHECK(decoder.Consume(Uart(1, 6, safe.data(), 25), 2000));
    CHECK(!decoder.Read().control.remote.robot_enable);
    dust_control::chassis::ChassisInputReader freshness;
    dust_control::chassis::ChassisCycleInput checked{};
    freshness.Read(sources, checked, 1000, .001, false);
    CHECK(checked.requested_enable);
    freshness.Read(sources, checked,
                   dust_control::chassis::InputDeadlines{}.remote_us + 2000,
                   .001, false);
    CHECK(!checked.remote_fresh && !checked.requested_enable);
    dust_control::chassis::ChassisInputReader imu_freshness;
    dust_control::chassis::ControllerInputs imu_sources{};
    imu_sources.imu.valid = true;
    imu_sources.imu.sequence = 1;
    imu_sources.imu.precise_timestamp_us = 10000;
    const uint64_t imu_timeout_us =
        dust_control::chassis::InputDeadlines{}.hi91_us;
    imu_freshness.Read(imu_sources, checked, 10000 + imu_timeout_us, .001,
                       false);
    CHECK(checked.imu_fresh);
    imu_freshness.Read(imu_sources, checked, 10001 + imu_timeout_us, .001,
                       false);
    CHECK(!checked.imu_fresh);
    imu_sources.imu.sequence = 2;
    imu_sources.imu.precise_timestamp_us = 20000;
    imu_freshness.Read(imu_sources, checked, 20000, .001, false);
    CHECK(checked.imu_fresh);
    dust_control::chassis::ChassisInputReader motor_reader;
    dust_control::chassis::ControllerInputs motor_sources{};
    motor_sources.motors[0].valid = motor_sources.motors[1].valid = true;
    motor_sources.motors[0].precise_timestamp_us = 10000;
    motor_sources.motors[1].precise_timestamp_us = 12000;
    motor_reader.Read(motor_sources, checked,
                      10000 + dust_control::chassis::InputDeadlines{}.motors[0],
                      .001, false);
    CHECK(!(checked.motor_stale_mask & 3));
    motor_reader.Read(motor_sources, checked,
                      10001 + dust_control::chassis::InputDeadlines{}.motors[0],
                      .001, false);
    CHECK((checked.motor_stale_mask & 3) == 1);  // 仅左轮超时。
    CHECK(!checked.feedback_valid && (checked.feedback_invalid_mask & 1));
    motor_sources.motors[0].precise_timestamp_us = 13000;
    motor_reader.Read(motor_sources, checked, 13001, .001, false);
    CHECK(!(checked.motor_stale_mask & 3));
    // 重连后应清空已发布状态和未完成的 SBUS 帧。
    decoder.Reset();
    CHECK(decoder.Consume(Uart(1, 0, sbus.data(), 10), 2000));
    decoder.Reset();
    CHECK(!decoder.Read().control.remote.robot_enable);
    CHECK(decoder.Consume(Uart(1, 0, sbus.data() + 10, 15), 2000));
    CHECK(!decoder.Read().control.remote.robot_enable);
    CHECK(decoder.Consume(Uart(1, 1, sbus.data(), 25), 2000));
    CHECK(decoder.Read().control.remote.robot_enable);
    std::array<uint8_t, 82> hi91{};
    hi91[0] = 0x5a;
    hi91[1] = 0xa5;
    dust_control::wire::Put16(hi91.data() + 2, 76);
    auto* payload = hi91.data() + 6;
    payload[0] = 0x91;
    dust_control::wire::Put32(payload + 8, 1);
    float pitch = 3.0F;
    std::memcpy(payload + 52, &pitch, 4);
    dust_control::wire::Put16(
        hi91.data() + 4,
        dust_control::protocols::CalculateHi91FrameCrc(76, payload));
    CHECK(decoder.Consume(Uart(2, 0, hi91.data(), 40), 2000));
    CHECK(decoder.Consume(Uart(2, 1, hi91.data() + 40, 40), 2000));
    CHECK(decoder.Consume(Uart(2, 2, hi91.data() + 80, 2), 2000));
    CHECK(decoder.Read().control.imu.valid &&
          decoder.Read().control.imu.pitch_deg == 3.0F);
    auto sequence = decoder.Read().control.imu.sequence;
    CHECK(decoder.Consume(Uart(2, 3, hi91.data(), 40), 3000));
    CHECK(decoder.Consume(Uart(2, 4, hi91.data() + 40, 40), 3000));
    CHECK(decoder.Consume(Uart(2, 5, hi91.data() + 80, 2), 3000));
    CHECK(decoder.Read().control.imu.sequence ==
          sequence);  // 重复传感器时间不能刷新状态。
    dust_control::wire::UsbInPacket can{};
    dust_control::wire::CanReceiveReport can_message{};
    can_message.header = {10, 7};
    can_message.frame_count = 1;
    can_message.frames[0] = {{3, 8, 0, 0x201, {}}, 0xfffffff0U, 0};
    CHECK(dust_control::wire::Encode(can_message, can));
    CHECK(decoder.Consume(can, 1000));
    CHECK(decoder.Read().control.motors[0].precise_timestamp_us == 974);
    dust_control::wire::Put32(can.data() + 4, 50000);
    CHECK(decoder.Consume(can, 60000));
    CHECK(!decoder.Read().control.motors[0].valid);
    // 以固定几何状态运行真实平衡控制器；重复更新并改变速度目标后，
    // 所有输出都必须保持有限。
    dust_control::chassis::BalanceController balance;
    input = {};
    input.dt = .001;
    input.imu_fresh = input.feedback_valid = true;
    input.leg.left.length = input.leg.right.length = .20;
    input.leg.left.angle = input.leg.right.angle = 1.5707963267948966;
    input.leg.left.jacobian[0][0] = input.leg.right.jacobian[0][0] = .1;
    input.leg.left.jacobian[1][1] = input.leg.right.jacobian[1][1] = .1;
    dust_control::chassis::ChassisControlOutput output{};
    for (unsigned tick = 0; tick < 1000; ++tick) {
        input.remote.chassis_x = tick > 500 ? .2 : 0;
        balance.Update(input, output);
        CHECK(std::isfinite(output.physical_wheel_torque.left));
        CHECK(std::isfinite(output.joint_torque.left.b));
    }

    // 动态电机配置提供轮力矩预算，平衡控制器不能继续使用编译期默认预算。
    dust_control::chassis::BalanceController no_turn(0.0);
    dust_control::chassis::BalanceController configured_turn(2.0);
    input.remote.chassis_x = 0;
    input.remote.chassis_rotate = .5;
    input.remote.run = input.requested_enable = true;
    input.theta.left = input.theta.right =
        dust_control::chassis::ChassisConfig{}.theta_balance_bias();
    dust_control::chassis::ChassisControlOutput disabled_turn{}, enabled_turn{};
    for (unsigned tick = 0; tick < 500; ++tick) {
        no_turn.Update(input, disabled_turn);
        configured_turn.Update(input, enabled_turn);
    }
    CHECK(std::abs(enabled_turn.allocated_turn_torque) > 0);
    CHECK(std::abs(enabled_turn.allocated_turn_torque) <= 2.0);
    CHECK(disabled_turn.allocated_turn_torque == 0);

    std::cout
        << "Session framing, disabled output, stale input, UART gaps, HI91 "
           "CRC, clock wrap and real control math passed\n";
}
