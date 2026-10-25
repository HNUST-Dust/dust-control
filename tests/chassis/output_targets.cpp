/* SPDX-License-Identifier: Apache-2.0 */
#include <cassert>

#include "../support/motor_fixture.hpp"
#include "chassis_config.h"
#include "chassis_output_policy.h"
#include "hardware/motors/motor_protocol.hpp"

int main() {
    using namespace dust_control::modules;
    // 冻结当前使能握手的时序合约，独立于实现中的私有常量。
    constexpr unsigned kExpectedArmTicks = 300;
    constexpr unsigned kExpectedModePeriodTicks = 10;
    constexpr unsigned kExpectedClearTicks = 50;
    ChassisOutputPolicy policy;
    dust_control::devices::MotorEncoder encoder(
        dust_control::test::MotorFixture());
    ChassisCycleInput input{};
    ChassisControlOutput output{};
    input.feedback_valid = input.imu_fresh = input.dm_ready =
        input.arm_complete = input.requested_enable = true;
    output.joint_control_active = true;

    assert(policy.Apply(ChassisControlState::kDisabled, 0, input, output));
    auto encoded = encoder.Encode(output.motor_targets);
    assert(encoded && encoded->count == 6);
    for (unsigned index = 2; index < 6; ++index) {
        assert(output.motor_targets[index].mode ==
               dust_control::devices::MotorMode::kDisable);
    }
    // 复用输出对象也不能残留上一周期的模式命令。
    policy.Apply(ChassisControlState::kDisabled, 1, input, output);
    encoded = encoder.Encode(output.motor_targets);
    assert(encoded && encoded->count == 0);
    policy.Apply(ChassisControlState::kDisabled, 10, input, output);
    encoded = encoder.Encode(output.motor_targets);
    assert(encoded && encoded->count == 2);

    for (unsigned tick = 0; tick < kExpectedArmTicks; ++tick) {
        policy.Apply(ChassisControlState::kDmArming, tick, input, output);
        assert(output.motor_targets[0].mode ==
               dust_control::devices::MotorMode::kTorque);
        assert(output.motor_targets[0].torque_nm == 0);
        if (tick % kExpectedModePeriodTicks == 0) {
            assert(output.motor_targets[2].mode ==
                   (tick < kExpectedClearTicks
                        ? dust_control::devices::MotorMode::kClearError
                        : dust_control::devices::MotorMode::kEnable));
        } else {
            assert(output.motor_targets[2].mode ==
                   dust_control::devices::MotorMode::kNone);
        }
    }
    assert(policy.arm_complete());
    policy.Apply(ChassisControlState::kSafetyStop, 0, input, output);
    assert(!policy.arm_complete());

    policy.Apply(ChassisControlState::kWaitingFeedback, 0, input, output);
    for (const auto& target : output.motor_targets) {
        assert(target.mode == dust_control::devices::MotorMode::kTorque &&
               target.torque_nm == 0);
    }
    output.joint_control_active = true;
    output.physical_wheel_torque = {.123, -.456};
    policy.Apply(ChassisControlState::kBalance, 0, input, output);
    assert(output.motor_targets[0].torque_nm == .123);
    assert(output.motor_targets[1].torque_nm == -.456);
    policy.Apply(ChassisControlState::kRecovery, 0, input, output);
    assert(output.motor_targets[0].torque_nm == 0);
    assert(output.motor_targets[1].torque_nm == 0);
}
