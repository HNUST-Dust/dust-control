/* SPDX-License-Identifier: Apache-2.0 */

#include "chassis/actions/recovery_controller.h"

#include <algorithm>
#include <cmath>

#include "chassis_config.h"
#include "chassis/leg/leg_joint_coordinates.h"

namespace {
// 本模块使用的固定算法参数，调整时与对应控制逻辑一起审查。
constexpr double kRecoveryUprightAccelSign = 1;
constexpr double kRecoveryUprightAccelThresholdG = 0.4;
constexpr double kRecoveryMaximumLegLength = 0.33;
constexpr double kRecoveryMinimumLegLength = 0.15;
constexpr double kRecoverySwingTheta = 1.45;
constexpr double kRecoveryTurnoverSweepRateDeg = 180.0;
constexpr double kRecoverySwingAngleRateDeg = 180.0;
constexpr double kRecoveryDrawbackAngleRateDeg = 200.0;
constexpr double kRecoveryExtendLegRate = 1.2;
constexpr double kRecoveryEscapeLegRate = 1.8;
constexpr double kRecoveryDrawbackLegRate = 1.2;
constexpr double kRecoveryTurnoverStuckSpeedDeg = 35.0;
constexpr double kRecoverySwingStuckSpeedDeg = 15.0;
constexpr uint32_t kRecoveryStuckTimeoutMs = 600;
constexpr uint32_t kRecoveryUprightHoldMs = 20;
constexpr uint32_t kRecoveryReadyHoldMs = 50;
constexpr uint32_t kRecoveryTurnoverTimeoutMs = 4000;
constexpr uint32_t kRecoverySwingTimeoutMs = 2500;
constexpr uint32_t kRecoveryDrawbackTimeoutMs = 1500;
constexpr uint32_t kRecoveryMaximumTurnoverReversals = 3;
}  // namespace

namespace {
using namespace dust_control::chassis::chassis_config;
constexpr double kRecoveryPitchLimit = 0.45;
constexpr double kRecoveryRollLimit = 20.0 * kDegToRad;

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

void RecoveryController::Reset() {
    phase_ = RecoveryPhase::kInit;
    phase_elapsed_s_ = ready_elapsed_s_ = upright_elapsed_s_ = 0.0;
    turnover_stuck_elapsed_s_ = turnover_progress_ = 0.0;
    turnover_direction_ = 1;
    turnover_reversals_ = 0U;
    reference_initialized_ = false;
    phase_start_angle_ = {};
    leg_length_reference_ = {};
    leg_angle_reference_ = {};
    swing_stuck_elapsed_s_ = {};
    swing_escape_phase_ = {};
    target_initialized_ = {};
    joint_target_ = {};
    joint_pid_ = {};
}

void RecoveryController::EnterPhase(RecoveryPhase phase,
                                    const ChassisCycleInput &input) {
    phase_ = phase;
    phase_elapsed_s_ = ready_elapsed_s_ = 0.0;
    for (Side side : {Side::kLeft, Side::kRight}) {
        phase_start_angle_.Get(side) = input.leg.Get(side).angle;
    }
}

RecoveryControllerOutput RecoveryController::Update(
    const ChassisCycleInput &input, RecoveryGimbalStatus gimbal) {
    const double dt = std::clamp(input.dt, 0.0005, 0.01);
    phase_elapsed_s_ += dt;
    if (!reference_initialized_) {
        for (Side side : {Side::kLeft, Side::kRight}) {
            leg_length_reference_.Get(side) = input.leg.Get(side).length;
            leg_angle_reference_.Get(side) = input.leg.Get(side).angle;
        }
        reference_initialized_ = true;
    }

    // 回正先更新参考及阶段，再用当前参考求解关节输出。
    UpdatePhase(input, gimbal, dt);
    const bool targets_valid = UpdateJointTargets(input);
    RecoveryControllerOutput output = {};
    output.phase = phase_;
    output.control.joint_control_active = targets_valid &&
                                          phase_ != RecoveryPhase::kFailed &&
                                          phase_ != RecoveryPhase::kReturn;
    output.request_gimbal_disable = phase_ != RecoveryPhase::kReturn;
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

    output.finished = phase_ == RecoveryPhase::kReturn;
    output.failed = phase_ == RecoveryPhase::kFailed;
    return output;
}

void RecoveryController::UpdatePhase(const ChassisCycleInput &input,
                                     const RecoveryGimbalStatus &gimbal,
                                     double dt) {
    switch (phase_) {
        case RecoveryPhase::kInit: {
            // 正值表示重力沿配置的机体直立方向。
            const double upright_accel_g =
                kRecoveryUprightAccelSign * input.imu.vertical_accel_g;
            if (!std::isfinite(input.imu.vertical_accel_g) ||
                std::abs(input.imu.vertical_accel_g) < 0.05) {
                EnterPhase(RecoveryPhase::kFailed, input);
            } else if (upright_accel_g > kRecoveryUprightAccelThresholdG) {
                // 已经直立时只收腿并对齐；此时进入 Swing 会把普通使能
                // 变成不必要的翻身动作。
                EnterPhase(RecoveryPhase::kDrawBack, input);
            } else {
                EnterPhase(RecoveryPhase::kTurnOver, input);
            }
            break;
        }
        case RecoveryPhase::kTurnOver:
            UpdateTurnOver(input, gimbal, dt);
            break;
        case RecoveryPhase::kYawFront:
            if (!gimbal.available ||
                (gimbal.aligned && gimbal.safe_for_leg_motion)) {
                EnterPhase(RecoveryPhase::kSwing, input);
            }
            break;
        case RecoveryPhase::kSwing:
            UpdateSwing(input, dt);
            break;
        case RecoveryPhase::kDrawBack:
            UpdateDrawBack(input, dt);
            break;
        case RecoveryPhase::kReturn:
        case RecoveryPhase::kFailed:
            break;
    }
}

void RecoveryController::UpdateTurnOver(const ChassisCycleInput &input,
                                        const RecoveryGimbalStatus &gimbal,
                                        double dt) {
    using namespace chassis_config;
    const double upright_accel_g =
        kRecoveryUprightAccelSign * input.imu.vertical_accel_g;
    const bool upright_detected =
        upright_accel_g > kRecoveryUprightAccelThresholdG;
    for (Side side : {Side::kLeft, Side::kRight}) {
        leg_length_reference_.Get(side) =
            MoveToward(leg_length_reference_.Get(side),
                       kRecoveryMaximumLegLength, kRecoveryExtendLegRate * dt);
    }
    // 检测到直立后立即冻结摆动命令。短确认窗口用于排除单样本冲击尖峰，
    // 避免像原先保持 100 ms 那样额外摆动 18 度。
    if (!upright_detected) {
        turnover_progress_ += turnover_direction_ *
                              kRecoveryTurnoverSweepRateDeg * kDegToRad * dt;
    }
    for (Side side : {Side::kLeft, Side::kRight}) {
        leg_angle_reference_.Get(side) =
            phase_start_angle_.Get(side) + turnover_progress_;
    }
    const bool legs_extended =
        input.leg.left.length > 0.30 && input.leg.right.length > 0.30;
    const bool stopped = !upright_detected && legs_extended &&
                         (std::abs(input.leg.left.angle_rate) <
                              kRecoveryTurnoverStuckSpeedDeg * kDegToRad ||
                          std::abs(input.leg.right.angle_rate) <
                              kRecoveryTurnoverStuckSpeedDeg * kDegToRad);
    turnover_stuck_elapsed_s_ = stopped ? turnover_stuck_elapsed_s_ + dt : 0.0;
    if (turnover_stuck_elapsed_s_ >=
        static_cast<double>(kRecoveryStuckTimeoutMs) * 0.001) {
        turnover_stuck_elapsed_s_ = 0.0;
        turnover_progress_ = 0.0;
        turnover_direction_ = -turnover_direction_;
        for (Side side : {Side::kLeft, Side::kRight}) {
            phase_start_angle_.Get(side) = input.leg.Get(side).angle;
        }
        if (++turnover_reversals_ > kRecoveryMaximumTurnoverReversals) {
            EnterPhase(RecoveryPhase::kFailed, input);
        }
    }
    upright_elapsed_s_ =
        upright_detected ? upright_elapsed_s_ + dt : upright_elapsed_s_ * 0.5;
    if (upright_elapsed_s_ >=
        static_cast<double>(kRecoveryUprightHoldMs) * 0.001) {
        // 无云台时没有额外的朝向约束，立即收腿也能抑制残余翻转运动。
        EnterPhase(gimbal.available ? RecoveryPhase::kYawFront
                                    : RecoveryPhase::kDrawBack,
                   input);
    } else if (phase_elapsed_s_ >=
               static_cast<double>(kRecoveryTurnoverTimeoutMs) * 0.001) {
        EnterPhase(RecoveryPhase::kFailed, input);
    }
}

void RecoveryController::UpdateSwing(const ChassisCycleInput &input,
                                     double dt) {
    using namespace chassis_config;
    bool all_ready = true;
    for (Side side : {Side::kLeft, Side::kRight}) {
        const double swing_target = input.pitch + kRecoverySwingTheta;
        const bool stuck =
            input.leg.Get(side).length > 0.30 &&
            std::abs(input.leg.Get(side).angle_rate) <
                kRecoverySwingStuckSpeedDeg * kDegToRad &&
            std::abs(std::remainder(input.leg.Get(side).angle - swing_target,
                                    kTwoPi)) > 10.0 * kDegToRad;
        swing_stuck_elapsed_s_.Get(side) =
            stuck ? swing_stuck_elapsed_s_.Get(side) + dt : 0.0;
        if (swing_stuck_elapsed_s_.Get(side) >=
            static_cast<double>(kRecoveryStuckTimeoutMs) * 0.001) {
            swing_escape_phase_.Get(side) = SwingEscapePhase::kBackToVertical;
            swing_stuck_elapsed_s_.Get(side) = 0.0;
        }
        if (swing_escape_phase_.Get(side) ==
            SwingEscapePhase::kBackToVertical) {
            leg_length_reference_.Get(side) = MoveToward(
                leg_length_reference_.Get(side), kRecoveryMinimumLegLength,
                kRecoveryEscapeLegRate * dt);
            leg_angle_reference_.Get(side) = MoveTowardAngle(
                leg_angle_reference_.Get(side), input.pitch + 0.5 * kTwoPi,
                kRecoverySwingAngleRateDeg * kDegToRad * dt);
            if (std::abs(std::remainder(
                    input.leg.Get(side).angle - input.pitch - 0.5 * kTwoPi,
                    kTwoPi)) < 10.0 * kDegToRad) {
                swing_escape_phase_.Get(side) = SwingEscapePhase::kToFinal;
            }
        } else {
            leg_length_reference_.Get(side) = MoveToward(
                leg_length_reference_.Get(side), kRecoveryMaximumLegLength,
                kRecoveryExtendLegRate * dt);
            leg_angle_reference_.Get(side) =
                MoveTowardAngle(leg_angle_reference_.Get(side), swing_target,
                                kRecoverySwingAngleRateDeg * kDegToRad * dt);
        }
        const bool side_ready =
            std::abs(std::remainder(input.leg.Get(side).angle - swing_target,
                                    kTwoPi)) < 15.0 * kDegToRad;
        all_ready = all_ready && side_ready;
    }
    const bool pose_ready =
        all_ready &&
        std::abs(input.pitch) < kRecoveryPitchLimit + 15.0 * kDegToRad &&
        std::abs(input.roll) < kRecoveryRollLimit + 10.0 * kDegToRad;
    ready_elapsed_s_ = pose_ready ? ready_elapsed_s_ + dt : 0.0;
    if (ready_elapsed_s_ >= static_cast<double>(kRecoveryReadyHoldMs) * 0.001) {
        EnterPhase(RecoveryPhase::kDrawBack, input);
    } else if (phase_elapsed_s_ >=
               static_cast<double>(kRecoverySwingTimeoutMs) * 0.001) {
        EnterPhase(RecoveryPhase::kFailed, input);
    }
}

void RecoveryController::UpdateDrawBack(const ChassisCycleInput &input,
                                        double dt) {
    using namespace chassis_config;
    for (Side side : {Side::kLeft, Side::kRight}) {
        leg_length_reference_.Get(side) = MoveToward(
            leg_length_reference_.Get(side), kRecoveryMinimumLegLength,
            kRecoveryDrawbackLegRate * dt);
        leg_angle_reference_.Get(side) =
            MoveTowardAngle(leg_angle_reference_.Get(side), input.pitch,
                            kRecoveryDrawbackAngleRateDeg * kDegToRad * dt);
    }
    const bool ready =
        std::abs(input.leg.left.length - kRecoveryMinimumLegLength) < 0.05 &&
        std::abs(input.leg.right.length - kRecoveryMinimumLegLength) < 0.05 &&
        std::abs(input.leg.left.length - input.leg.right.length) < 0.04 &&
        std::abs(input.leg.left.angle - input.pitch) < 15.0 * kDegToRad &&
        std::abs(input.leg.right.angle - input.pitch) < 15.0 * kDegToRad &&
        std::abs(input.pitch) < kRecoveryPitchLimit &&
        std::abs(input.roll) < kRecoveryRollLimit;
    ready_elapsed_s_ = ready ? ready_elapsed_s_ + dt : 0.0;
    if (ready_elapsed_s_ >= static_cast<double>(kRecoveryReadyHoldMs) * 0.001) {
        EnterPhase(RecoveryPhase::kReturn, input);
    } else if (phase_elapsed_s_ >=
               static_cast<double>(kRecoveryDrawbackTimeoutMs) * 0.001) {
        EnterPhase(RecoveryPhase::kFailed, input);
    }
}

bool RecoveryController::UpdateJointTargets(const ChassisCycleInput &input) {
    bool all_valid = true;
    for (Side side : {Side::kLeft, Side::kRight}) {
        const LegHardwareMap hardware =
            config_.leg_hardware(side == Side::kRight);
        const double raw_angle =
            RawLegAngle(leg_angle_reference_.Get(side), hardware);
        const double hx = leg_length_reference_.Get(side) * std::sin(raw_angle);
        const double hz =
            -leg_length_reference_.Get(side) * std::cos(raw_angle);
        const double seed1 = target_initialized_.Get(side)
                                 ? joint_target_.Get(side).d
                                 : input.joint_position.Get(side).d;
        const double seed2 = target_initialized_.Get(side)
                                 ? joint_target_.Get(side).b
                                 : input.joint_position.Get(side).b;
        double phi1 = 0.0;
        double phi2 = 0.0;
        if (!InverseKinematics(hx, hz, hardware.kinematic_branch, seed1, seed2,
                               phi1, phi2, config_)) {
            all_valid = false;
            continue;
        }
        joint_target_.Get(side).b = phi2;
        joint_target_.Get(side).d = phi1;
        target_initialized_.Get(side) = true;
    }

    return all_valid && target_initialized_.left && target_initialized_.right;
}

double RecoveryController::ComputeJointTorque(Side side, Joint joint,
                                              double target, double position,
                                              double velocity, double dt) {
    using namespace chassis_config;
    if (!std::isfinite(target) || !std::isfinite(position) ||
        !std::isfinite(velocity)) {
        return 0.0;
    }

    JointPidState &pid = joint_pid_.Get(side).Get(joint);
    const double error = std::remainder(target - position, kTwoPi);
    pid.position_integral = std::clamp(pid.position_integral + error * dt,
                                       -config_.joint_position_max_speed,
                                       config_.joint_position_max_speed);
    const double speed_target = std::clamp(
        config_.joint_position_kp * error +
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
