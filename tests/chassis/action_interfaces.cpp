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
#include "chassis/jump_controller.h"
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
    enum class Scenario {
        kUpright,
        kGimbal,
        kInvalidAcceleration,
        kTurnoverTimeout,
        kDrawbackTimeout,
        kTurnoverStuck,
        kSwingTimeout,
        kSwingEscape
    };
    for (auto scenario :
         {Scenario::kUpright, Scenario::kGimbal, Scenario::kInvalidAcceleration,
          Scenario::kTurnoverTimeout, Scenario::kDrawbackTimeout,
          Scenario::kTurnoverStuck, Scenario::kSwingTimeout,
          Scenario::kSwingEscape}) {
        chassis::RecoveryController recovery;
        chassis::ChassisCycleInput input{};
        input.pitch = 0.03;
        input.roll = -0.02;
        SetPose(input, 0.18, 0.22, 0.05, -0.06);
        chassis::RecoveryGimbalStatus gimbal;
        gimbal.available = scenario == Scenario::kGimbal ||
                           (scenario == Scenario::kSwingTimeout ||
                            scenario == Scenario::kSwingEscape);
        gimbal.aligned = gimbal.safe_for_leg_motion = false;
        bool terminal = false;
        unsigned swing_ticks = 0;
        visited[size_t(recovery.phase())] = true;
        for (unsigned tick = 0; tick < 4500; ++tick) {
            input.imu.vertical_accel_g = 1;
            if (scenario == Scenario::kInvalidAcceleration) {
                input.imu.vertical_accel_g = 0;
            } else if (scenario == Scenario::kTurnoverTimeout ||
                       scenario == Scenario::kTurnoverStuck ||
                       (gimbal.available && tick < 30)) {
                input.imu.vertical_accel_g = -1;
            }
            if (scenario == Scenario::kUpright && tick >= 60) {
                SetPose(input, 0.15, 0.15, input.pitch, input.pitch);
            }
            if (scenario == Scenario::kTurnoverStuck) {
                SetPose(input, 0.33, 0.33, 0, 0);
            }
            if (gimbal.available) {
                gimbal.aligned = gimbal.safe_for_leg_motion = tick >= 80;
                if (recovery.phase() == chassis::RecoveryPhase::kSwing) {
                    ++swing_ticks;
                    double angle = input.pitch + 1.45;
                    double length = 0.33;
                    if (scenario == Scenario::kSwingTimeout ||
                        (scenario == Scenario::kSwingEscape &&
                         swing_ticks < 650)) {
                        angle = 0;
                    } else if (scenario == Scenario::kSwingEscape &&
                               swing_ticks < 700) {
                        angle =
                            input.pitch + chassis::chassis_config::kTwoPi * 0.5;
                        length = 0.15;
                    }
                    SetPose(input, length, length, angle, angle);
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
                assert(result.finished == ((scenario == Scenario::kUpright ||
                                            scenario == Scenario::kGimbal) ||
                                           scenario == Scenario::kSwingEscape));
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
    enum class Scenario { kSuccess, kMotion1Timeout, kMotion2Timeout };
    for (auto scenario : {Scenario::kSuccess, Scenario::kMotion1Timeout,
                          Scenario::kMotion2Timeout}) {
        chassis::ClimbStairsController climb;
        chassis::ChassisCycleInput input{};
        input.pitch = 0.03;
        SetPose(input, 0.18, 0.22, 0.05, -0.06);
        bool terminal = false;
        visited[size_t(climb.phase())] = true;
        for (unsigned tick = 0; tick < 1500; ++tick) {
            if (scenario != Scenario::kMotion1Timeout && tick >= 40) {
                if (climb.phase() == chassis::ClimbStairsPhase::kMotion1) {
                    SetPose(input, 0.33, 0.33, input.pitch + 1.45,
                            input.pitch + 1.45);
                } else if (scenario == Scenario::kSuccess &&
                           climb.phase() ==
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
            Trace(trace, 20 + int(scenario), int(result.phase), result.finished,
                  result.failed, false, result.control);
            if (result.finished || result.failed) {
                assert(!result.control.joint_control_active);
                assert(result.finished == (scenario == Scenario::kSuccess));
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
void JumpPaths(bool trace) {
    std::array<bool, 7> visited{};
    enum class Scenario {
        kSuccess,
        kThrustTimeout,
        kRetractTimeout,
        kAirTimeout,
        kTotalTimeout
    };
    for (auto scenario : {Scenario::kSuccess, Scenario::kThrustTimeout,
                          Scenario::kRetractTimeout, Scenario::kAirTimeout,
                          Scenario::kTotalTimeout}) {
        chassis::JumpController jump;
        chassis::ChassisCycleInput input{};
        input.pitch = 0.03;
        input.theta = {0.05, -0.04};
        input.theta_rate = {0.02, -0.03};
        input.imu.yaw_deg = 175;
        unsigned thrust_ticks = 0, air_ticks = 0;
        bool terminal = false;
        visited[size_t(jump.phase())] = true;
        for (unsigned tick = 0; tick < 2200; ++tick) {
            const auto phase = jump.phase();
            if (phase == chassis::JumpPhase::kInit ||
                phase == chassis::JumpPhase::kPreCompress) {
                const double length =
                    scenario == Scenario::kTotalTimeout || tick < 20 ? 0.22
                                                                     : 0.15;
                SetPose(input, length, length, 0.05, -0.04);
            } else if (phase == chassis::JumpPhase::kThrust) {
                ++thrust_ticks;
                const double length =
                    scenario == Scenario::kThrustTimeout ||
                            (scenario == Scenario::kTotalTimeout &&
                             thrust_ticks < 250)
                        ? 0.22
                        : 0.335;
                SetPose(input, length, length, 0.05, -0.04);
            } else if (phase == chassis::JumpPhase::kAirRetract) {
                const double length =
                    scenario == Scenario::kRetractTimeout ? 0.24 : 0.16;
                SetPose(input, length, length, 0.05, -0.04);
            } else if (phase == chassis::JumpPhase::kAir) {
                ++air_ticks;
                SetPose(input, 0.20, 0.21, 0.05, -0.04);
                input.leg.left.length_rate = input.leg.right.length_rate =
                    scenario == Scenario::kSuccess ? -0.1 : 0.0;
            }
            const auto result = jump.Update(input);
            visited[size_t(result.phase)] = true;
            assert(result.phase == jump.phase());
            assert(result.finished ==
                   (result.phase == chassis::JumpPhase::kReturn));
            assert(result.failed ==
                   (result.phase == chassis::JumpPhase::kFailed));
            if (phase == chassis::JumpPhase::kInit ||
                phase == chassis::JumpPhase::kPreCompress) {
                assert(result.precompress_leg_length.has_value());
                assert(!result.control.bypass_gas_spring_compensation);
            } else {
                assert(!result.precompress_leg_length.has_value());
                assert(result.control.bypass_gas_spring_compensation ==
                       (phase == chassis::JumpPhase::kThrust));
            }
            // 直接动作接口由总控设置控制许可，此处只生成物理输出。
            assert(!result.control.joint_control_active);
            for (double value : {result.control.physical_wheel_torque.left,
                                 result.control.physical_wheel_torque.right,
                                 result.control.body_on_leg_torque.left,
                                 result.control.body_on_leg_torque.right,
                                 result.control.leg_axial_force.left,
                                 result.control.leg_axial_force.right}) {
                assert(std::isfinite(value));
            }
            Trace(trace, 30 + int(scenario), int(result.phase), result.finished,
                  result.failed, false, result.control);
            if (trace) {
                std::cout << "jump "
                          << result.precompress_leg_length.has_value() << ' '
                          << result.precompress_leg_length.value_or(0) << ' '
                          << result.control.bypass_gas_spring_compensation
                          << ' ' << result.control.physical_wheel_torque.left
                          << ' ' << result.control.physical_wheel_torque.right
                          << ' ' << result.control.body_on_leg_torque.left
                          << ' ' << result.control.body_on_leg_torque.right
                          << ' ' << result.control.leg_axial_force.left << ' '
                          << result.control.leg_axial_force.right << '\n';
            }
            if (result.finished || result.failed) {
                assert(result.finished == (scenario == Scenario::kSuccess));
                if (scenario == Scenario::kAirTimeout) {
                    assert(air_ticks >= 1500);
                }
                if (scenario == Scenario::kTotalTimeout) {
                    assert(air_ticks < 1500 && tick >= 1999);
                }
                terminal = true;
                break;
            }
        }
        assert(terminal);
        jump.Reset();
        assert(jump.phase() == chassis::JumpPhase::kInit);
        SetPose(input, 0.15, 0.15, 0.05, -0.04);
        const auto restarted = jump.Update(input);
        assert(restarted.phase == chassis::JumpPhase::kThrust);
        assert(restarted.precompress_leg_length.has_value());
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
        Trace(trace, 40, 0, false, false, false, control);
    }
}
}  // namespace
int main(int argc, char** argv) {
    const bool trace = argc == 2 && std::string_view(argv[1]) == "--trace";
    std::cout << std::setprecision(17);
    RecoveryPaths(trace);
    ClimbPaths(trace);
    JointMapping(trace);
    JumpPaths(trace);
}
