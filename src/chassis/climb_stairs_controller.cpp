/* SPDX-License-Identifier: Apache-2.0 */

#include "climb_stairs_controller.h"

#include <algorithm>
#include <cmath>

#include "chassis_config.h"

namespace {
// 本模块使用的固定算法参数，调整时与对应控制逻辑一起审查。
constexpr double kClimbIntermediateLegLength = 0.33;
constexpr double kClimbRetractedLegLength = 0.15;
constexpr double kClimbLegLengthRate = 1.8;
constexpr double kClimbSwingTheta = 1.45;
constexpr double kClimbMotion1AngleRateDeg = 300.0;
constexpr double kClimbMotion2AngleRateDeg = 270.0;
constexpr double kClimbLengthTolerance = 0.05;
constexpr double kClimbMotion1AngleToleranceDeg = 20.0;
constexpr double kClimbMotion2AngleToleranceDeg = 15.0;
constexpr uint32_t kClimbReadyHoldMs = 30;
constexpr uint32_t kClimbMotion1TimeoutMs = 1000;
constexpr uint32_t kClimbMotion2TimeoutMs = 800;
}  // namespace

namespace {
using namespace dust_control::chassis::chassis_config;

double MoveToward(double current, double target, double maximum_delta) {
    return current +
           std::clamp(target - current, -maximum_delta, maximum_delta);
}

double MoveTowardAngle(double current, double target, double maximum_delta) {
    const double error = std::remainder(target - current, kTwoPi);
    return std::remainder(
        current + std::clamp(error, -maximum_delta, maximum_delta), kTwoPi);
}
}  // namespace

namespace dust_control::chassis {

void ClimbStairsController::Reset() {
    phase_ = ClimbStairsPhase::kInit;
    phase_elapsed_s_ = 0.0;
    ready_elapsed_s_ = 0.0;
    reference_initialized_ = false;
    target_initialized_ = {};
    leg_length_reference_ = {};
    leg_angle_reference_ = {};
    joint_target_ = {};
    joint_pid_ = {};
}

void ClimbStairsController::EnterPhase(ClimbStairsPhase phase) {
    phase_ = phase;
    phase_elapsed_s_ = 0.0;
    ready_elapsed_s_ = 0.0;
}

ClimbStairsControllerOutput ClimbStairsController::Update(
    const ChassisCycleInput &input) {
    const double dt = std::clamp(input.dt, 0.0005, 0.01);
    phase_elapsed_s_ += dt;
    if (!reference_initialized_) {
        for (Side side : {Side::kLeft, Side::kRight}) {
            leg_length_reference_.Get(side) = input.leg.Get(side).length;
            leg_angle_reference_.Get(side) = input.leg.Get(side).angle;
        }
        reference_initialized_ = true;
        EnterPhase(ClimbStairsPhase::kMotion1);
    }

    // 爬楼先更新参考及阶段，再求解关节目标和 PID 输出。
    UpdatePhase(input, dt);
    const bool targets_valid = UpdateJointTargets(input);

    ClimbStairsControllerOutput output = {};
    output.phase = phase_;
    output.control.joint_control_active = targets_valid &&
                                          phase_ != ClimbStairsPhase::kFailed &&
                                          phase_ != ClimbStairsPhase::kReturn;
    if (targets_valid) {
        for (Side side : {Side::kLeft, Side::kRight}) {
            for (Joint joint : {Joint::kB, Joint::kD}) {
                output.control.joint_torque.Get(side).Get(joint) =
                    ComputeJointTorque(
                        side, joint, joint_target_.Get(side).Get(joint),
                        input.joint_position.Get(side).Get(joint),
                        input.joint_velocity.Get(side).Get(joint), dt);
            }
        }
    }

    output.finished = phase_ == ClimbStairsPhase::kReturn;
    output.failed = phase_ == ClimbStairsPhase::kFailed;
    return output;
}

void ClimbStairsController::UpdatePhase(const ChassisCycleInput &input,
                                        double dt) {
    switch (phase_) {
        case ClimbStairsPhase::kMotion1:
            UpdateMotion1(input, dt);
            break;
        case ClimbStairsPhase::kMotion2:
            UpdateMotion2(input, dt);
            break;
        case ClimbStairsPhase::kInit:
        case ClimbStairsPhase::kReturn:
        case ClimbStairsPhase::kFailed:
            break;
    }
}

void ClimbStairsController::UpdateMotion1(const ChassisCycleInput &input,
                                          double dt) {
    using namespace chassis_config;
    for (Side side : {Side::kLeft, Side::kRight}) {
        leg_length_reference_.Get(side) =
            MoveToward(leg_length_reference_.Get(side),
                       kClimbIntermediateLegLength, kClimbLegLengthRate * dt);
        leg_angle_reference_.Get(side) = MoveTowardAngle(
            leg_angle_reference_.Get(side), input.pitch + kClimbSwingTheta,
            kClimbMotion1AngleRateDeg * kDegToRad * dt);
    }
    const bool ready =
        std::abs(input.leg.left.length - kClimbIntermediateLegLength) <
            kClimbLengthTolerance &&
        std::abs(input.leg.right.length - kClimbIntermediateLegLength) <
            kClimbLengthTolerance &&
        std::abs(std::remainder(
            input.leg.left.angle - input.pitch - kClimbSwingTheta, kTwoPi)) <
            kClimbMotion1AngleToleranceDeg * kDegToRad &&
        std::abs(std::remainder(
            input.leg.right.angle - input.pitch - kClimbSwingTheta, kTwoPi)) <
            kClimbMotion1AngleToleranceDeg * kDegToRad;
    ready_elapsed_s_ = ready ? ready_elapsed_s_ + dt : 0.0;
    if (ready_elapsed_s_ >= static_cast<double>(kClimbReadyHoldMs) * 0.001) {
        EnterPhase(ClimbStairsPhase::kMotion2);
    } else if (phase_elapsed_s_ >=
               static_cast<double>(kClimbMotion1TimeoutMs) * 0.001) {
        EnterPhase(ClimbStairsPhase::kFailed);
    }
}

void ClimbStairsController::UpdateMotion2(const ChassisCycleInput &input,
                                          double dt) {
    using namespace chassis_config;
    for (Side side : {Side::kLeft, Side::kRight}) {
        leg_length_reference_.Get(side) =
            MoveToward(leg_length_reference_.Get(side),
                       kClimbRetractedLegLength, kClimbLegLengthRate * dt);
        leg_angle_reference_.Get(side) =
            MoveTowardAngle(leg_angle_reference_.Get(side), input.pitch,
                            kClimbMotion2AngleRateDeg * kDegToRad * dt);
    }
    const bool ready =
        std::abs(input.leg.left.length - kClimbRetractedLegLength) <
            kClimbLengthTolerance &&
        std::abs(input.leg.right.length - kClimbRetractedLegLength) <
            kClimbLengthTolerance &&
        std::abs(std::remainder(input.leg.left.angle - input.pitch, kTwoPi)) <
            kClimbMotion2AngleToleranceDeg * kDegToRad &&
        std::abs(std::remainder(input.leg.right.angle - input.pitch, kTwoPi)) <
            kClimbMotion2AngleToleranceDeg * kDegToRad;
    ready_elapsed_s_ = ready ? ready_elapsed_s_ + dt : 0.0;
    if (ready_elapsed_s_ >= static_cast<double>(kClimbReadyHoldMs) * 0.001) {
        EnterPhase(ClimbStairsPhase::kReturn);
    } else if (phase_elapsed_s_ >=
               static_cast<double>(kClimbMotion2TimeoutMs) * 0.001) {
        EnterPhase(ClimbStairsPhase::kFailed);
    }
}

bool ClimbStairsController::UpdateJointTargets(const ChassisCycleInput &input) {
    bool all_valid = true;
    for (Side side : {Side::kLeft, Side::kRight}) {
        const double raw_angle =
            RawKinematicAngle(side, leg_angle_reference_.Get(side));
        const double target_hx =
            leg_length_reference_.Get(side) * std::sin(raw_angle);
        const double target_hz =
            -leg_length_reference_.Get(side) * std::cos(raw_angle);
        const double seed_phi1 = target_initialized_.Get(side)
                                     ? joint_target_.Get(side).d
                                     : input.joint_position.Get(side).d;
        const double seed_phi2 = target_initialized_.Get(side)
                                     ? joint_target_.Get(side).b
                                     : input.joint_position.Get(side).b;
        double target_phi1 = 0.0;
        double target_phi2 = 0.0;
        if (!InverseKinematics(
                target_hx, target_hz,
                config_.leg_hardware(side == Side::kRight).kinematic_branch,
                seed_phi1, seed_phi2, target_phi1, target_phi2, config_)) {
            all_valid = false;
            continue;
        }
        joint_target_.Get(side).b = target_phi2;
        joint_target_.Get(side).d = target_phi1;
        target_initialized_.Get(side) = true;
    }

    return all_valid && target_initialized_.left && target_initialized_.right;
}

double ClimbStairsController::RawKinematicAngle(Side side,
                                                double normalized_angle) {
    return side == Side::kLeft
               ? std::remainder(
                     config_.left_leg_angle_offset - normalized_angle, kTwoPi)
               : std::remainder(
                     normalized_angle + config_.right_leg_angle_offset, kTwoPi);
}

double ClimbStairsController::ComputeJointTorque(Side side, Joint joint,
                                                 double target, double position,
                                                 double velocity, double dt) {
    using namespace chassis_config;
    if (!std::isfinite(target) || !std::isfinite(position) ||
        !std::isfinite(velocity)) {
        return 0.0;
    }

    JointPidState &pid = joint_pid_.Get(side).Get(joint);
    const double position_error = std::remainder(target - position, kTwoPi);
    pid.position_integral = std::clamp(
        pid.position_integral + position_error * dt,
        -config_.joint_position_max_speed, config_.joint_position_max_speed);
    const double speed_target = std::clamp(
        config_.joint_position_kp * position_error +
            kJointPositionKi * pid.position_integral,
        -config_.joint_position_max_speed, config_.joint_position_max_speed);
    const double speed_error = speed_target - velocity;
    pid.speed_integral =
        std::clamp(pid.speed_integral + speed_error * dt,
                   -kJointIntegralTorqueLimit, kJointIntegralTorqueLimit);
    return std::clamp(config_.joint_speed_kp * speed_error +
                          kJointSpeedKi * pid.speed_integral,
                      -kJointTorqueMax, kJointTorqueMax);
}

}  // namespace dust_control::chassis
