/* SPDX-License-Identifier: Apache-2.0 */
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string_view>

#include "chassis/control/chassis_input_reader.h"
#include "chassis/control/chassis_output_policy.h"
#include "chassis/leg/gas_spring.h"
#include "chassis/leg/leg_joint_coordinates.h"
#include "chassis/leg/leg_vmc.h"

namespace chassis = dust_control::chassis;
namespace {
void Near(double actual, double expected) {
    assert(std::abs(actual - expected) < 1e-10);
}
chassis::ChassisCycleInput ReadInput(bool trace,
                                     const chassis::ChassisConfig& config) {
    chassis::ControllerInputs sources{};
    constexpr uint64_t now_us = 1000000;
    for (auto& motor : sources.motors) {
        motor.valid = motor.enabled = true;
        motor.precise_timestamp_us = now_us;
    }
    sources.remote.sequence = sources.imu.sequence = 1;
    sources.remote.robot_enable = true;
    sources.imu.valid = true;
    sources.imu.precise_timestamp_us = now_us;
    sources.imu.pitch_deg = 12;
    sources.imu.roll_deg = -7;
    sources.imu.pitch_rate_rad_s = 0.23;
    sources.imu.roll_rate_rad_s = -0.31;
    sources.motors[0].speed_rpm = 73;
    sources.motors[1].speed_rpm = -91;
    for (bool right : {false, true}) {
        const auto hardware = config.leg_hardware(right);
        const double length = right ? 0.27 : 0.21;
        const double angle = right ? -0.2 : 0.1;
        // 测试输入独立构造原始几何，不调用待验证的腿角转换。
        const double raw = right ? angle + config.right_leg_angle_offset
                                 : config.left_leg_angle_offset - angle;
        double d = 0, b = 0;
        bool solved = false;
        for (int sd = -3; sd <= 3 && !solved; ++sd) {
            for (int sb = -3; sb <= 3 && !solved; ++sb) {
                solved = chassis::InverseKinematics(
                    length * std::sin(raw), -length * std::cos(raw),
                    hardware.kinematic_branch, sd, sb, d, b, config);
            }
        }
        assert(solved);
        auto& motor_b = sources.motors[right ? 4 : 2];
        auto& motor_d = sources.motors[right ? 5 : 3];
        motor_b.position_rad = b;
        motor_d.position_rad = d;
        motor_b.velocity_rad_s = right ? -0.3 : 0.1;
        motor_d.velocity_rad_s = right ? 0.4 : -0.2;
        motor_b.torque_nm = right ? 3 : 1;
        motor_d.torque_nm = right ? -4 : -2;
    }
    chassis::ChassisInputReader reader({}, config);
    chassis::ChassisCycleInput input{};
    reader.Read(sources, input, now_us, 0.001, true);
    assert(input.feedback_valid && input.requested_enable && input.dm_ready &&
           input.arm_complete);
    Near(input.pitch, 12 * chassis::chassis_config::kDegToRad);
    Near(input.roll, -7 * chassis::chassis_config::kDegToRad);
    Near(input.pitch_rate, sources.imu.pitch_rate_rad_s);
    Near(input.roll_rate, sources.imu.roll_rate_rad_s);
    assert(input.wheel_motor_speed_rpm.left == 73 &&
           input.wheel_motor_speed_rpm.right == -91);
    for (auto side : {chassis::Side::kLeft, chassis::Side::kRight}) {
        const bool right = side == chassis::Side::kRight;
        const auto& b = sources.motors[right ? 4 : 2];
        const auto& d = sources.motors[right ? 5 : 3];
        const auto& leg = input.leg.Get(side);
        chassis::LegKinematics raw{};
        assert(chassis::ComputeLegKinematics(
            d.position_rad, b.position_rad, d.velocity_rad_s, b.velocity_rad_s,
            config.leg_hardware(right).kinematic_branch, raw, config));
        const double expected_angle =
            right ? std::remainder(raw.angle - config.right_leg_angle_offset,
                                   chassis::chassis_config::kTwoPi)
                  : -std::remainder(raw.angle - config.left_leg_angle_offset,
                                    chassis::chassis_config::kTwoPi);
        assert(leg.hx == raw.hx && leg.hz == raw.hz &&
               leg.length == raw.length);
        assert(leg.length_rate == raw.length_rate);
        Near(leg.angle, expected_angle);
        Near(leg.angle_rate, right ? raw.angle_rate : -raw.angle_rate);
        Near(input.theta.Get(side),
             std::remainder(expected_angle - input.pitch,
                            chassis::chassis_config::kTwoPi));
        Near(input.theta_rate.Get(side), leg.angle_rate - input.pitch_rate);
        for (unsigned row = 0; row < 2; ++row) {
            for (unsigned column = 0; column < 2; ++column) {
                assert(leg.jacobian[row][column] == raw.jacobian[row][column]);
            }
        }
        assert(input.joint_position.Get(side).b == b.position_rad);
        assert(input.joint_position.Get(side).d == d.position_rad);
        assert(input.joint_velocity.Get(side).b == b.velocity_rad_s);
        assert(input.joint_velocity.Get(side).d == d.velocity_rad_s);
        assert(input.joint_torque_feedback.Get(side).b == b.torque_nm);
        assert(input.joint_torque_feedback.Get(side).d == d.torque_nm);
        Near(chassis::JointBMotorAngle(
                 chassis::JointBLinkAngle(b.position_rad, config), config),
             b.position_rad);
        Near(chassis::JointDMotorAngle(
                 chassis::JointDLinkAngle(d.position_rad, config), config),
             d.position_rad);
        auto vmc_config = config;
        vmc_config.leg_length_kp = vmc_config.leg_length_kd = 0;
        // 雅可比列仍为 [D, B]，校准腿角不应改变原始几何的虚功关系。
        const auto vmc =
            chassis::ComputeLegVmc(leg, leg.length, 40, 0, 2, 0, 0, vmc_config);
        Near(vmc.phi1_torque * d.velocity_rad_s +
                 vmc.phi2_torque * b.velocity_rad_s,
             vmc.axial_force * raw.length_rate + 2 * raw.angle_rate);
        if (trace) {
            std::cout << right << ' ' << input.pitch << ' ' << input.roll << ' '
                      << leg.angle << ' ' << leg.angle_rate << ' ' << leg.hx
                      << ' ' << leg.hz << ' ' << input.theta.Get(side) << ' '
                      << input.theta_rate.Get(side) << ' ' << vmc.phi1_torque
                      << ' ' << vmc.phi2_torque << '\n';
        }
    }
    return input;
}
void OutputBoundary(bool trace, const chassis::ChassisCycleInput& input,
                    chassis::ChassisConfig config) {
    for (double limit : {54.0, 3.0}) {
        config.joint_torque_limit = limit;
        chassis::ChassisOutputPolicy policy(config);
        for (auto state : {chassis::ChassisControlState::kBalance,
                           chassis::ChassisControlState::kRecovery,
                           chassis::ChassisControlState::kClimbStairs,
                           chassis::ChassisControlState::kJump,
                           chassis::ChassisControlState::kFlight}) {
            for (bool bypass : {false, true}) {
                chassis::ChassisControlOutput output{};
                output.joint_control_active = true;
                output.bypass_gas_spring_compensation = bypass;
                output.joint_torque = {{0.2, -0.3}, {0.4, -0.5}};
                output.physical_wheel_torque = {0.12, -0.34};
                const auto before = output.joint_torque;
                policy.Apply(state, 0, input, output);
                for (auto side :
                     {chassis::Side::kLeft, chassis::Side::kRight}) {
                    const bool right = side == chassis::Side::kRight;
                    const auto raw = chassis::ComputeGasSpringCompensation(
                        input.leg.Get(side),
                        input.joint_position.Get(side).d +
                            config.joint_d_zero_offset,
                        config.length_ag, config.length_gh,
                        right ? config.gas_spring_right_upper_offset
                              : config.gas_spring_left_upper_offset,
                        config.gas_spring_lower_distance,
                        right ? config.gas_spring_right_force
                              : config.gas_spring_left_force);
                    const bool skip =
                        state == chassis::ChassisControlState::kJump && bypass;
                    for (auto joint :
                         {chassis::Joint::kB, chassis::Joint::kD}) {
                        const double compensation =
                            skip ? 0 : raw[joint == chassis::Joint::kB ? 1 : 0];
                        const double expected = std::clamp(
                            before.Get(side).Get(joint) + compensation, -limit,
                            limit);
                        Near(output.joint_torque.Get(side).Get(joint),
                             expected);
                        const unsigned slot =
                            (right ? 4 : 2) +
                            (joint == chassis::Joint::kD ? 1 : 0);
                        assert(output.motor_targets[slot].torque_nm ==
                               output.joint_torque.Get(side).Get(joint));
                        if (trace) {
                            std::cout << int(state) << ' ' << limit << ' '
                                      << bypass << ' ' << slot << ' '
                                      << expected << '\n';
                        }
                    }
                }
                const bool wheels =
                    state == chassis::ChassisControlState::kBalance ||
                    state == chassis::ChassisControlState::kJump ||
                    state == chassis::ChassisControlState::kFlight;
                assert(output.motor_targets[0].torque_nm ==
                       (wheels ? 0.12 : 0));
                assert(output.motor_targets[1].torque_nm ==
                       (wheels ? -0.34 : 0));
            }
        }
    }
}
void OutputSafetyGates(const chassis::ChassisCycleInput& valid_input,
                       const chassis::ChassisConfig& config) {
    chassis::ChassisOutputPolicy policy(config);
    for (auto gate : {&chassis::ChassisCycleInput::feedback_valid,
                      &chassis::ChassisCycleInput::imu_fresh,
                      &chassis::ChassisCycleInput::dm_ready,
                      &chassis::ChassisCycleInput::arm_complete,
                      &chassis::ChassisCycleInput::requested_enable}) {
        auto input = valid_input;
        input.*gate = false;
        chassis::ChassisControlOutput output{};
        output.joint_control_active = true;
        output.joint_torque = {{1, 2}, {3, 4}};
        policy.Apply(chassis::ChassisControlState::kBalance, 0, input, output);
        for (unsigned slot = 2; slot < 6; ++slot) {
            assert(output.motor_targets[slot].torque_nm == 0);
        }
    }
    chassis::ChassisControlOutput inactive{};
    inactive.joint_torque = {{1, 2}, {3, 4}};
    policy.Apply(chassis::ChassisControlState::kBalance, 0, valid_input,
                 inactive);
    for (unsigned slot = 2; slot < 6; ++slot) {
        assert(inactive.motor_targets[slot].torque_nm == 0);
    }
    // 无效控制量必须归零，不能生成只有弹簧补偿的目标。
    chassis::ChassisControlOutput invalid{};
    invalid.joint_control_active = true;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    invalid.joint_torque = {{nan, inf}, {-inf, nan}};
    policy.Apply(chassis::ChassisControlState::kBalance, 0, valid_input,
                 invalid);
    for (unsigned slot = 2; slot < 6; ++slot) {
        assert(invalid.motor_targets[slot].torque_nm == 0);
    }
}
}  // namespace
int main(int argc, char** argv) {
    const bool trace = argc == 2 && std::string_view(argv[1]) == "--trace";
    std::cout << std::setprecision(17);
    chassis::ChassisConfig config;
    config.joint_b_zero_offset = 0.17;
    config.joint_d_zero_offset = 2.9;
    config.left_leg_angle_offset = 0.13;
    config.right_leg_angle_offset = -2.8;
    config.gas_spring_left_force = 17;
    config.gas_spring_right_force = 29;
    const auto input = ReadInput(trace, config);
    OutputBoundary(trace, input, config);
    OutputSafetyGates(input, config);
}
