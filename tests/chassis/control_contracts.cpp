/* SPDX-License-Identifier: Apache-2.0 */
#include <cassert>
#include <cmath>

#include "../support/motor_fixture.hpp"
#include "chassis/balance_controller.h"
#include "chassis/chassis_controller.h"
#include "hardware/motors/motor_protocol.hpp"
#define CHECK(condition) assert(condition)
int main() {
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
}
