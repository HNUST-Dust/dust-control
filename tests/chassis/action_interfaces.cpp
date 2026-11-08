/* SPDX-License-Identifier: Apache-2.0 */
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string_view>

#include "chassis/chassis_types.h"
#include "chassis/climb_stairs_controller.h"
#include "chassis/recovery_controller.h"

namespace chassis = dust_control::chassis;
namespace {
void SetPose(chassis::ChassisCycleInput& input, double left_length,
             double right_length, double left_angle, double right_angle,
             const chassis::ChassisConfig& config = {}) {
    for (auto side : {chassis::Side::kLeft, chassis::Side::kRight}) {
        const bool right = side == chassis::Side::kRight;
        const auto hardware = config.leg_hardware(right);
        const double length = right ? right_length : left_length;
        const double angle = right ? right_angle : left_angle;
        const double raw_angle =
            hardware.leg_angle_offset + hardware.leg_coordinate_sign * angle;
        double d = 0, b = 0;
        bool solved = false;
        for (int seed_d = -3; seed_d <= 3 && !solved; ++seed_d) {
            for (int seed_b = -3; seed_b <= 3 && !solved; ++seed_b) {
                solved = chassis::InverseKinematics(
                    length * std::sin(raw_angle), -length * std::cos(raw_angle),
                    hardware.kinematic_branch, seed_d, seed_b, d, b, config);
            }
        }
        assert(solved);
        input.joint_position.Get(side) = {b, d};
        auto& leg = input.leg.Get(side);
        assert(chassis::ComputeLegKinematics(
            d, b, 0, 0, hardware.kinematic_branch, leg, config));
        leg.angle = hardware.leg_coordinate_sign *
                    std::remainder(leg.angle - hardware.leg_angle_offset,
                                   chassis::chassis_config::kTwoPi);
    }
}
void CheckControl(const chassis::ChassisControlOutput& control) {
    assert(control.physical_wheel_torque.left == 0);
    assert(control.physical_wheel_torque.right == 0);
    for (const auto& target : control.motor_targets) {
        assert(target.mode == dust_control::devices::MotorMode::kNone);
    }
    for (auto side : {chassis::Side::kLeft, chassis::Side::kRight}) {
        for (auto joint : {chassis::Joint::kB, chassis::Joint::kD}) {
            assert(std::isfinite(control.joint_torque.Get(side).Get(joint)));
        }
    }
}
void Trace(bool trace, int action, int phase, bool finished, bool failed,
           bool gimbal_disable, const chassis::ChassisControlOutput& control) {
    if (trace) {
        std::cout << action << ' ' << phase << ' ' << finished << ' ' << failed
                  << ' ' << gimbal_disable << ' '
                  << control.joint_control_active << ' '
                  << control.joint_torque.left.b << ' '
                  << control.joint_torque.left.d << ' '
                  << control.joint_torque.right.b << ' '
                  << control.joint_torque.right.d << '\n';
    }
}
void RecoveryPaths(bool trace) {
    std::array<bool, 7> visited{};
    for (unsigned scenario = 0; scenario < 5; ++scenario) {
        chassis::RecoveryController recovery;
        chassis::ChassisCycleInput input{};
        input.pitch = 0.03;
        input.roll = -0.02;
        SetPose(input, 0.18, 0.22, 0.05, -0.06);
        chassis::RecoveryGimbalStatus gimbal;
        gimbal.available = scenario == 1;
        gimbal.aligned = gimbal.safe_for_leg_motion = false;
        bool terminal = false;
        visited[size_t(recovery.phase())] = true;
        for (unsigned tick = 0; tick < 4500; ++tick) {
            input.imu.vertical_accel_g =
                scenario == 2                                   ? 0
                : scenario == 3 || (scenario == 1 && tick < 30) ? -1
                                                                : 1;
            if (scenario == 0 && tick >= 60) {
                SetPose(input, 0.15, 0.15, input.pitch, input.pitch);
            }
            if (scenario == 1) {
                gimbal.aligned = gimbal.safe_for_leg_motion = tick >= 80;
                if (recovery.phase() == chassis::RecoveryPhase::kSwing) {
                    SetPose(input, 0.33, 0.33, input.pitch + 1.45,
                            input.pitch + 1.45);
                } else if (recovery.phase() ==
                           chassis::RecoveryPhase::kDrawBack) {
                    SetPose(input, 0.15, 0.15, input.pitch, input.pitch);
                }
            }
            const auto result = recovery.Update(input, gimbal);
            CheckControl(result.control);
            visited[size_t(result.phase)] = true;
            assert(result.phase == recovery.phase());
            assert(result.request_gimbal_disable ==
                   (result.phase != chassis::RecoveryPhase::kReturn));
            assert(result.finished ==
                   (result.phase == chassis::RecoveryPhase::kReturn));
            assert(result.failed ==
                   (result.phase == chassis::RecoveryPhase::kFailed));
            Trace(trace, int(scenario), int(result.phase), result.finished,
                  result.failed, result.request_gimbal_disable, result.control);
            if (result.finished || result.failed) {
                assert(!result.control.joint_control_active);
                assert(result.finished == (scenario < 2));
                terminal = true;
                break;
            }
        }
        assert(terminal);
        recovery.Reset();
        assert(recovery.phase() == chassis::RecoveryPhase::kInit);
    }
    assert(std::all_of(visited.begin(), visited.end(),
                       [](bool value) { return value; }));
}
void ClimbPaths(bool trace) {
    std::array<bool, 5> visited{};
    for (unsigned scenario = 0; scenario < 2; ++scenario) {
        chassis::ClimbStairsController climb;
        chassis::ChassisCycleInput input{};
        input.pitch = 0.03;
        SetPose(input, 0.18, 0.22, 0.05, -0.06);
        bool terminal = false;
        visited[size_t(climb.phase())] = true;
        for (unsigned tick = 0; tick < 1500; ++tick) {
            if (scenario == 0 && tick >= 40) {
                if (climb.phase() == chassis::ClimbStairsPhase::kMotion1) {
                    SetPose(input, 0.33, 0.33, input.pitch + 1.45,
                            input.pitch + 1.45);
                } else if (climb.phase() ==
                           chassis::ClimbStairsPhase::kMotion2) {
                    SetPose(input, 0.15, 0.15, input.pitch, input.pitch);
                }
            }
            const auto result = climb.Update(input);
            CheckControl(result.control);
            visited[size_t(result.phase)] = true;
            assert(result.phase == climb.phase());
            assert(result.finished ==
                   (result.phase == chassis::ClimbStairsPhase::kReturn));
            assert(result.failed ==
                   (result.phase == chassis::ClimbStairsPhase::kFailed));
            Trace(trace, 5 + int(scenario), int(result.phase), result.finished,
                  result.failed, false, result.control);
            if (result.finished || result.failed) {
                assert(!result.control.joint_control_active);
                assert(result.finished == (scenario == 0));
                terminal = true;
                break;
            }
        }
        assert(terminal);
        climb.Reset();
        assert(climb.phase() == chassis::ClimbStairsPhase::kInit);
    }
    assert(std::all_of(visited.begin(), visited.end(),
                       [](bool value) { return value; }));
}
void JointMapping(bool trace) {
    chassis::ChassisConfig config;
    config.joint_position_kp = 0;
    chassis::ChassisCycleInput input{};
    input.imu.vertical_accel_g = 1;
    SetPose(input, 0.18, 0.22, 0.05, -0.06, config);
    input.joint_velocity = {{0.1, -0.2}, {0.3, -0.4}};
    chassis::RecoveryController recovery(config);
    chassis::ClimbStairsController climb(config);
    const chassis::RecoveryGimbalStatus gimbal{};
    const auto recovery_result = recovery.Update(input, gimbal);
    const auto climb_result = climb.Update(input);
    for (const auto& control :
         {recovery_result.control, climb_result.control}) {
        assert(control.joint_control_active);
        CheckControl(control);
        for (auto side : {chassis::Side::kLeft, chassis::Side::kRight}) {
            for (auto joint : {chassis::Joint::kB, chassis::Joint::kD}) {
                const double expected =
                    -config.joint_speed_kp *
                    input.joint_velocity.Get(side).Get(joint);
                assert(std::abs(control.joint_torque.Get(side).Get(joint) -
                                expected) < 1e-12);
            }
        }
        Trace(trace, 7, 0, false, false, false, control);
    }
}
}  // namespace
int main(int argc, char** argv) {
    const bool trace = argc == 2 && std::string_view(argv[1]) == "--trace";
    std::cout << std::setprecision(17);
    RecoveryPaths(trace);
    ClimbPaths(trace);
    JointMapping(trace);
}
