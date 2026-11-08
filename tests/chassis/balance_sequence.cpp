/* SPDX-License-Identifier: Apache-2.0 */
#include <array>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string_view>

#include "chassis/balance_controller.h"

namespace chassis = dust_control::chassis;
namespace {
// 同时观察力矩、运动诊断和公开参考，避免为测试暴露私有状态。
std::array<double, 24> Values(const chassis::BalanceController& controller,
                              const chassis::ChassisControlOutput& output) {
    return {output.physical_wheel_torque.left,
            output.physical_wheel_torque.right,
            output.requested_body_on_leg_torque.left,
            output.requested_body_on_leg_torque.right,
            output.body_on_leg_torque.left,
            output.body_on_leg_torque.right,
            output.leg_axial_force.left,
            output.leg_axial_force.right,
            output.joint_torque.left.b,
            output.joint_torque.left.d,
            output.joint_torque.right.b,
            output.joint_torque.right.d,
            output.raw_body_speed,
            output.estimated_body_speed,
            output.measured_turn_wheel_speed,
            output.requested_turn_torque,
            output.allocated_turn_torque,
            output.differential_leg_torque,
            output.roll_reference,
            output.roll_compensation_force,
            controller.target_leg_length(),
            controller.target_yaw_rate(),
            controller.target_yaw_angle(),
            controller.filtered_yaw_rate()};
}

chassis::ChassisCycleInput ResetInput() {
    chassis::ChassisCycleInput input{};
    input.dt = 0.001;
    input.requested_enable = input.remote.run = true;
    input.remote.chassis_x = 0.6;
    input.remote.chassis_rotate = -0.4;
    input.imu.yaw_deg = 179;
    input.imu.yaw_rate_rad_s = 0.3;
    input.wheel_motor_speed_rpm = {120, -90};
    input.leg.left.length = 0.19;
    input.leg.right.length = 0.27;
    input.theta = {0.04, -0.08};
    input.common_theta = 0.12;
    input.pitch = -0.09;
    input.roll = 0.16;
    for (auto side : {chassis::Side::kLeft, chassis::Side::kRight}) {
        auto& leg = input.leg.Get(side);
        leg.jacobian[0][0] = 0.08;
        leg.jacobian[0][1] = -0.06;
        leg.jacobian[1][0] = 0.04;
        leg.jacobian[1][1] = 0.05;
    }
    return input;
}

void ResetContracts() {
    // 从不同历史出发，三个入口都应清除估计、运动参考、yaw 和积分历史。
    enum class Entry { kHistory, kEnable, kSynchronize };
    for (auto entry : {Entry::kHistory, Entry::kEnable, Entry::kSynchronize}) {
        chassis::ChassisConfig config;
        config.leg_length_ki = 250;
        chassis::BalanceController warm(2.0, config), fresh(2.0, config);
        warm.ResetForEnable();
        fresh.ResetForEnable();
        warm.set_target_leg_length(0.24);
        fresh.set_target_leg_length(0.24);
        auto input = ResetInput();
        for (unsigned i = 0; i < 300; ++i) {
            chassis::ChassisControlOutput output{};
            warm.Update(input, output);
        }
        assert(warm.target_yaw_rate() != 0);
        assert(warm.filtered_yaw_rate() != 0);
        if (entry == Entry::kHistory) {
            warm.Reset();
            assert(warm.target_leg_length() == 0.24);
        } else if (entry == Entry::kEnable) {
            warm.ResetForEnable();
            fresh.ResetForEnable();
            assert(warm.target_leg_length() == config.default_leg_length);
        } else {
            warm.SynchronizeReferences(input);
            fresh.SynchronizeReferences(input);
            assert(warm.target_leg_length() ==
                   0.5 * (input.leg.left.length + input.leg.right.length));
        }
        assert(warm.target_yaw_rate() == 0);
        assert(warm.target_yaw_angle() == 0);
        assert(warm.filtered_yaw_rate() == 0);
        // 重复同一个入口不应改变结果，随后验证 yaw 从新采样初始化。
        if (entry == Entry::kHistory) {
            warm.Reset();
        } else if (entry == Entry::kEnable) {
            warm.ResetForEnable();
        } else {
            warm.SynchronizeReferences(input);
        }
        input.imu.yaw_deg = -175;
        input.imu.yaw_rate_rad_s = -0.7;
        input.remote.chassis_x = input.remote.chassis_rotate = 0;
        for (unsigned i = 0; i < 20; ++i) {
            chassis::ChassisControlOutput actual{}, expected{};
            warm.Update(input, actual);
            fresh.Update(input, expected);
            assert(Values(warm, actual) == Values(fresh, expected));
            if (i == 0) {
                assert(warm.filtered_yaw_rate() == input.imu.yaw_rate_rad_s);
                assert(std::abs(warm.target_yaw_angle() -
                                input.imu.yaw_deg *
                                    chassis::chassis_config::kDegToRad) <
                       1e-12);
            }
        }
    }
    // 零时长探测让交接姿态不发生渐变，直接验证 Reset 保留非零姿态参考。
    chassis::BalanceController controller;
    auto pose = ResetInput();
    pose.dt = 0;
    pose.requested_enable = pose.remote.run = false;
    controller.SynchronizeReferences(pose);
    chassis::ChassisControlOutput before{};
    controller.Update(pose, before);
    const auto expected = Values(controller, before);
    auto disturbed = pose;
    disturbed.wheel_motor_speed_rpm = {-600, 420};
    disturbed.imu.yaw_deg = -90;
    for (unsigned i = 0; i < 30; ++i) {
        chassis::ChassisControlOutput output{};
        controller.Update(disturbed, output);
    }
    controller.Reset();
    chassis::ChassisControlOutput after{};
    controller.Update(pose, after);
    assert(after.roll_reference == pose.roll);
    assert(expected == Values(controller, after));
}

void Run(bool trace, double wheel_budget) {
    chassis::ChassisConfig config;
    config.leg_length_ki = 250.0;
    chassis::BalanceController controller(wheel_budget, config);
    chassis::ChassisCycleInput input{};
    input.feedback_valid = input.imu_fresh = true;
    for (unsigned tick = 0; tick < 3600; ++tick) {
        const double t = tick * 0.001;
        input.dt = tick % 3 == 0 ? 0.002 : 0.001;
        input.requested_enable = tick % 700 < 650;
        input.remote.run = tick % 900 < 800;
        input.remote.chassis_x = std::sin(3 * t);
        input.remote.chassis_rotate = std::cos(5 * t);
        input.remote.leg_length_delta = std::sin(7 * t);
        input.pitch = 0.07 * std::sin(2 * t);
        input.pitch_rate = 0.14 * std::cos(2 * t);
        input.roll = 0.12 * std::sin(t);
        input.common_theta = 0.06 * std::cos(t);
        input.theta = {0.09 * std::sin(t), -0.05 * std::cos(t)};
        input.theta_rate = {0.09 * std::cos(t), 0.05 * std::sin(t)};
        input.wheel_motor_speed_rpm = {180 * std::sin(t), 240 * std::cos(t)};
        input.imu.yaw_deg = std::remainder(175 + 30 * t, 360.0);
        input.imu.yaw_rate_rad_s = 0.3 * std::sin(9 * t);
        for (auto side : {chassis::Side::kLeft, chassis::Side::kRight}) {
            auto& leg = input.leg.Get(side);
            const bool right = side == chassis::Side::kRight;
            leg.length =
                right ? 0.24 + 0.02 * std::cos(t) : 0.19 + 0.03 * std::sin(t);
            leg.length_rate = right ? -0.02 * std::sin(t) : 0.03 * std::cos(t);
            leg.angle_rate = right ? -0.04 : 0.03;
            leg.jacobian[0][0] = right ? -0.07 : 0.08;
            leg.jacobian[0][1] = right ? 0.05 : -0.06;
            leg.jacobian[1][0] = right ? -0.03 : 0.04;
            leg.jacobian[1][1] = right ? -0.06 : 0.05;
        }
        if (tick == 0 || tick == 2400) {
            controller.ResetForEnable();
            assert(controller.target_leg_length() == config.default_leg_length);
            assert(controller.target_yaw_rate() == 0);
            assert(controller.filtered_yaw_rate() == 0);
        } else if (tick == 1200) {
            controller.SynchronizeReferences(input);
            assert(controller.target_leg_length() ==
                   0.5 * (input.leg.left.length + input.leg.right.length));
        } else if (tick == 1600) {
            const double length = controller.target_leg_length();
            controller.Reset();
            assert(controller.target_leg_length() == length);
            assert(controller.target_yaw_angle() == 0);
        } else if (tick == 2800) {
            controller.set_target_leg_length(0.15);
        }
        chassis::ExternalSupportForceCommand support;
        support.left_force_n = 45 + 10 * std::sin(t);
        support.right_force_n = 65 + 8 * std::cos(t);
        const bool external =
            (tick >= 600 && tick < 1000) || (tick >= 1800 && tick < 2100);
        const double previous_length = controller.target_leg_length();
        chassis::ChassisControlOutput output{};
        controller.Update(input, output, external ? &support : nullptr);
        if (!input.requested_enable || !input.remote.run) {
            assert(controller.target_leg_length() == previous_length);
        }
        assert(controller.target_leg_length() >=
               config.min_commanded_leg_length);
        assert(controller.target_leg_length() <=
               config.max_commanded_leg_length);
        if (external) {
            assert(output.roll_compensation_force ==
                   0.5 * (support.left_force_n - support.right_force_n));
        }
        assert(std::abs(output.allocated_turn_torque) <= wheel_budget);
        for (auto side : {chassis::Side::kLeft, chassis::Side::kRight}) {
            const auto& torque = output.joint_torque.Get(side);
            assert(std::isfinite(torque.b) && std::isfinite(torque.d));
            assert(std::abs(torque.b) <= config.joint_torque_limit);
            assert(std::abs(torque.d) <= config.joint_torque_limit);
        }
        if (trace) {
            std::cout << tick << ' ' << wheel_budget;
        }
        for (double value : Values(controller, output)) {
            assert(std::isfinite(value));
            if (trace) {
                std::cout << ' ' << value;
            }
        }
        if (trace) {
            std::cout << '\n';
        }
    }
}
}  // namespace
int main(int argc, char** argv) {
    const bool trace = argc == 2 && std::string_view(argv[1]) == "--trace";
    std::cout << std::setprecision(17);
    ResetContracts();
    Run(trace, 0.0);
    Run(trace, 2.0);
}
