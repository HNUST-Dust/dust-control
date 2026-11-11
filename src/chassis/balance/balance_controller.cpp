/* SPDX-License-Identifier: Apache-2.0 */

#include "chassis/balance/balance_controller.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "chassis/balance/balance_lqr14.h"
#include "chassis_config.h"
#include "chassis/leg/leg_vmc.h"
#include "chassis/balance/generated/lqr14_gain_samples.h"

namespace {
// 本模块使用的固定算法参数，调整时与对应控制逻辑一起审查。
constexpr double kBalanceThetaReferenceRate = 1.0;
constexpr double kBalancePitchReferenceRate = 0.6;
constexpr double kPositionErrorLimit = 0.15;
constexpr double kYawRateFilterCutoffHz = 20.0;
constexpr double kDifferentialLegTorqueLimit = 4.0;
constexpr double kRollReferenceRateDegPerSec = 30.0;
constexpr double kRollCompensationSign = 1;
}  // namespace

namespace dust_control::chassis {
namespace {
using namespace chassis_config;

double MoveToward(double current, double target, double max_delta) {
    return current + std::clamp(target - current, -max_delta, max_delta);
}

double ApplyRemoteDeadband(double value, const ChassisConfig &config) {
    if (!std::isfinite(value)) {
        return 0.0;
    }

    const double limited = std::clamp(value, -1.0, 1.0);
    const double magnitude = std::abs(limited);
    if (magnitude <= config.remote_deadband) {
        return 0.0;
    }

    return std::copysign(
        (magnitude - config.remote_deadband) / (1.0 - config.remote_deadband),
        limited);
}


}  // namespace

BalanceController::BalanceController(double wheel_torque_limit,
                                     ChassisConfig config)
    : config_(config),
      wheel_torque_limit_(wheel_torque_limit),
      target_leg_length_(config_.default_leg_length) {
    {
        ValidateChassisConfig(config_);
        lqr14_schedule_.emplace(Lqr14DevelopmentSamples());
        const auto trim = lqr14_schedule_->Evaluate(config_.default_leg_length);
        if (trim.has_pitch_trim && std::abs(trim.trim_anchor_theta_deg-config_.theta_balance_bias_deg) > 1e-8) {
            throw std::invalid_argument("LQR14 trim anchor differs from configured theta bias");
        }
    }
}

void BalanceController::Reset() {
    // target_leg_length_ 和三项姿态参考由交接入口管理，此处有意保留。
    body_motion_estimator_.Reset();
    target_body_speed_ = 0.0;
    target_body_position_ = 0.0;

    target_yaw_rate_ = 0.0;
    target_yaw_angle_ = 0.0;
    yaw_reference_initialized_ = false;

    filtered_yaw_rate_ = 0.0;
    yaw_rate_filter_initialized_ = false;

    side_state_ = {};
    lqr14_support_elapsed_s_ = 0.0;
    lqr14_blend_ = 0.0;
    lqr14_height_initialized_ = false;
    lqr14_filtered_height_rate_ = 0.0;
    lqr14_previous_height_reference_ = 0.0;
}

void BalanceController::ResetForEnable() {
    target_leg_length_ = config_.default_leg_length;
    balance_theta_reference_ = config_.theta_balance_bias();
    balance_pitch_reference_ = 0.0;
    balance_roll_reference_ = 0.0;
    Reset();
}

void BalanceController::SynchronizeReferences(const ChassisCycleInput &input) {
    target_leg_length_ = 0.5 * (input.leg.left.length + input.leg.right.length);
    balance_theta_reference_ = input.common_theta;
    balance_pitch_reference_ = input.pitch;
    balance_roll_reference_ = input.roll;
    Reset();
}

void BalanceController::Update(
    const ChassisCycleInput &input, ChassisControlOutput &output,
    const ExternalSupportForceCommand *external_support, bool allow_lqr14) {
    UpdateAttitudeReferences(input, external_support, output);
    const BodyMotionState &motion = EstimateBodyMotion(input, output);
    const MotionTracking tracking = UpdateMotionReferences(input, motion);
    output.motion = {.valid = true,
                     .raw_speed = output.raw_body_speed,
                     .estimated_speed = motion.speed,
                     .target_speed = target_body_speed_,
                     .estimated_position = motion.position,
                     .target_position = target_body_position_,
                     .unclamped_position_error = motion.position - target_body_position_,
                     .position_error = tracking.position_error,
                     .speed_error = tracking.speed_error,
                     .pitch_reference = balance_pitch_reference_,
                     .theta_reference = balance_theta_reference_,
                     .roll_reference = balance_roll_reference_};
    {
        ChassisControlOutput candidate = output;
        candidate.motion = output.motion;
        candidate.joint_control_active = output.joint_control_active;
        if (!ComputeLqr14Output(input, tracking, candidate, external_support, allow_lqr14)) {
            candidate.physical_wheel_torque = {};
            candidate.joint_torque = {};
            candidate.joint_control_active = false;
        }
        output = candidate;
        return;
    }
}

void BalanceController::UpdateAttitudeReferences(
    const ChassisCycleInput &input,
    const ExternalSupportForceCommand *external_support,
    ChassisControlOutput &output) {
    // 从交接姿态以受限速率回到稳态目标，角速度反馈始终保持完整控制权。
    double theta_target = config_.theta_balance_bias();
    if (lqr14_schedule_) {
        const auto trim = lqr14_schedule_->Evaluate(
            0.5*(input.leg.left.length+input.leg.right.length));
        if (trim.valid && !trim.clamped && trim.has_pitch_trim) {
            theta_target = trim.theta_equilibrium_rad;
        }
    }
    balance_theta_reference_ =
        MoveToward(balance_theta_reference_, theta_target,
                   kBalanceThetaReferenceRate * input.dt);
    balance_pitch_reference_ = MoveToward(
        balance_pitch_reference_, 0.0, kBalancePitchReferenceRate * input.dt);
    balance_roll_reference_ =
        MoveToward(balance_roll_reference_, 0.0,
                   kRollReferenceRateDegPerSec * kDegToRad * input.dt);
    output.roll_reference = balance_roll_reference_;
    const double roll_error = balance_roll_reference_ - input.roll;
    output.roll_compensation_force = std::clamp(
        kRollCompensationSign * config_.roll_compensation_kp * roll_error,
        -config_.roll_compensation_force_limit,
        config_.roll_compensation_force_limit);
    if (external_support != nullptr) {
        output.roll_compensation_force =
            0.5 *
            (external_support->left_force_n - external_support->right_force_n);
    }
}

const BodyMotionState &BalanceController::EstimateBodyMotion(
    const ChassisCycleInput &input, ChassisControlOutput &output) {
    // 结合轮速和腿部运动补偿估计机体前向速度。
    const SidePair<double> sin_theta = {
        .left = std::sin(input.theta.left),
        .right = std::sin(input.theta.right),
    };

    const SidePair<double> cos_theta = {
        .left = std::cos(input.theta.left),
        .right = std::cos(input.theta.right),
    };

    const SidePair<double> raw_wheel_omega = {
        .left = input.wheel_motor_speed_rpm.left * kRpmToRadPerSec /
                config_.wheel_reduction,
        .right = input.wheel_motor_speed_rpm.right * kRpmToRadPerSec /
                 config_.wheel_reduction,
    };

    // 将轮速转换到世界坐标，再叠加髋部随腿运动产生的速度。
    const SidePair<double> wheel_world_omega = {
        .left = config_.leg_hardware(false).wheel_feedback_sign *
                    raw_wheel_omega.left +
                input.pitch_rate - input.leg.left.angle_rate,
        .right = config_.leg_hardware(true).wheel_feedback_sign *
                     raw_wheel_omega.right +
                 input.pitch_rate - input.leg.right.angle_rate,
    };

    SidePair<double> body_speed;
    const auto calculate_body_speed = [&](Side side) {
        const LegKinematics &leg = input.leg.Get(side);
        body_speed.Get(side) =
            -config_.wheel_radius * wheel_world_omega.Get(side) +
            leg.length_rate * sin_theta.Get(side) +
            leg.length * input.theta_rate.Get(side) * cos_theta.Get(side);
    };

    calculate_body_speed(Side::kLeft);
    calculate_body_speed(Side::kRight);
    const double raw_common_x_speed =
        0.5 * (body_speed.left + body_speed.right);
    // 正值表示左轮向前、右轮向后，与遥控器横向通道的正方向一致。
    output.measured_turn_wheel_speed =
        0.5 * (body_speed.left - body_speed.right);
    output.raw_body_speed = raw_common_x_speed;
    // IMU 前向加速度轴尚未标定，暂传 0，避免引入错误的轴向和符号。
    const BodyMotionState &motion =
        body_motion_estimator_.Update(raw_common_x_speed, 0.0, input.dt);
    output.estimated_body_speed = motion.speed;
    return motion;
}

BalanceController::MotionTracking BalanceController::UpdateMotionReferences(
    const ChassisCycleInput &input, const BodyMotionState &motion) {
    const bool motion_command_enabled =
        input.remote.run && input.requested_enable;
    const double body_command =
        motion_command_enabled
            ? ApplyRemoteDeadband(input.remote.chassis_x, config_)
            : 0.0;
    const double turn_command =
        motion_command_enabled
            ? ApplyRemoteDeadband(input.remote.chassis_rotate, config_)
            : 0.0;
    const double leg_length_command =
        motion_command_enabled
            ? ApplyRemoteDeadband(input.remote.leg_length_delta, config_)
            : 0.0;
    const double previous_target_leg_length = target_leg_length_;
    target_leg_length_ = std::clamp(
        target_leg_length_ +
            leg_length_command * kLegLengthCommandRate * input.dt,
        config_.min_commanded_leg_length, config_.max_commanded_leg_length);
    const double target_leg_length_rate =
        input.dt > 0.0
            ? (target_leg_length_ - previous_target_leg_length) / input.dt
            : 0.0;
    target_body_speed_ =
        MoveToward(target_body_speed_, body_command * config_.max_body_speed,
                   config_.body_acceleration_limit * input.dt);
    /*
     * 当前遥控正转向使左轮向前、右轮向后，对应顺时针 yaw。
     * IMU 坐标以逆时针为正，因此目标 yaw-rate 在这里取反号，
     * 保持操作手现有的摇杆方向不变。
     */
    target_yaw_rate_ =
        MoveToward(target_yaw_rate_, -turn_command * config_.max_yaw_rate(),
                   config_.yaw_acceleration_limit() * input.dt);
    const double measured_yaw =
        static_cast<double>(input.imu.yaw_deg) * kDegToRad;
    if (!yaw_reference_initialized_) {
        target_yaw_angle_ = measured_yaw;
        yaw_reference_initialized_ = true;
    }

    target_yaw_angle_ =
        std::remainder(target_yaw_angle_ + target_yaw_rate_ * input.dt, kTwoPi);
    const double raw_yaw_rate = static_cast<double>(input.imu.yaw_rate_rad_s);
    if (!yaw_rate_filter_initialized_) {
        filtered_yaw_rate_ = raw_yaw_rate;
        yaw_rate_filter_initialized_ = true;
    } else {
        const double yaw_rate_filter_alpha =
            1.0 - std::exp(-kTwoPi * kYawRateFilterCutoffHz * input.dt);
        filtered_yaw_rate_ +=
            yaw_rate_filter_alpha * (raw_yaw_rate - filtered_yaw_rate_);
    }

    const double yaw_rate_error = filtered_yaw_rate_ - target_yaw_rate_;
    const double yaw_tracking_error =
        std::clamp(std::remainder(measured_yaw - target_yaw_angle_, kTwoPi),
                   -kYawTrackingErrorLimit, kYawTrackingErrorLimit);
    target_body_position_ += target_body_speed_ * input.dt;
    const double position_error =
        std::clamp(motion.position - target_body_position_,
                   -kPositionErrorLimit, kPositionErrorLimit);

    return {.position_error = position_error,
            .speed_error = motion.speed - target_body_speed_,
            .yaw_error = yaw_tracking_error,
            .yaw_rate_error = yaw_rate_error,
            .target_leg_length_rate = target_leg_length_rate};
}

void BalanceController::AllocateBalanceTorques(
    double left_wheel, double right_wheel, double left_leg, double right_leg,
    ChassisControlOutput &output) const {
    const double common_wheel_torque = std::isfinite(left_wheel + right_wheel)
                                           ? 0.5 * (left_wheel + right_wheel)
                                           : 0.0;
    output.requested_turn_torque =
        std::isfinite(left_wheel - right_wheel)
            ? std::clamp(0.5 * (left_wheel - right_wheel),
                         -config_.turn_torque_limit, config_.turn_torque_limit)
            : 0.0;
    output.differential_leg_torque =
        std::isfinite(left_leg - right_leg)
            ? std::clamp(0.5 * (left_leg - right_leg),
                         -kDifferentialLegTorqueLimit,
                         kDifferentialLegTorqueLimit)
            : 0.0;
    // 差动力矩只使用共模平衡力矩之外的余量，避免转向命令削弱平衡控制权。
    const double turn_torque_available =
        std::max(0.0, wheel_torque_limit_ - std::abs(common_wheel_torque));
    output.allocated_turn_torque =
        std::clamp(output.requested_turn_torque, -turn_torque_available,
                   turn_torque_available);
    // 正轮差模力矩仍是左加右减，与既有执行器坐标完全一致。
    output.physical_wheel_torque.left =
        common_wheel_torque + output.allocated_turn_torque;
    output.physical_wheel_torque.right =
        common_wheel_torque - output.allocated_turn_torque;
    const double common_leg_torque = std::isfinite(left_leg + right_leg)
                                         ? 0.5 * (left_leg + right_leg)
                                         : 0.0;
    output.requested_body_on_leg_torque.left =
        common_leg_torque + output.differential_leg_torque;
    output.requested_body_on_leg_torque.right =
        common_leg_torque - output.differential_leg_torque;
    output.body_on_leg_torque = output.requested_body_on_leg_torque;
}

bool BalanceController::ComputeLqr14Output(const ChassisCycleInput &input,
                                           const MotionTracking &tracking,
                                           ChassisControlOutput &candidate,
                                           const ExternalSupportForceCommand *external_support,
                                           bool allow_extensions) {
    const auto saved_side_state = side_state_;
    candidate.lqr14 = {};
    auto &diag = candidate.lqr14;
    const auto height = EstimateFlatGroundHipHeight(input, config_);
    diag.height_valid = height.valid;
    diag.height_m = height.height_m;
    diag.height_rate_m_s = height.rate_m_s;
    diag.ground_mismatch_m = height.ground_mismatch_m;
    if (!input.feedback_valid || !input.imu_fresh || !std::isfinite(input.dt) || input.dt < 0.0005 ||
        input.dt > 0.002) {
        diag.status = Lqr14Status::kInvalidFeedback;
        lqr14_support_elapsed_s_ = 0.0;
        lqr14_height_initialized_ = false;
        return false;
    }
    const double model_roll = config_.lqr14_roll_sign * input.roll;
    const double model_roll_rate = config_.lqr14_roll_sign * input.roll_rate;
    const double mean_length =
        0.5 * (input.leg.left.length + input.leg.right.length);
    const auto evaluation = lqr14_schedule_->Evaluate(mean_length);
    diag.endpoint_gain = evaluation.clamped;
    if (!evaluation.valid) {
        diag.status = Lqr14Status::kNumericalFailure;
        return false;
    }
    const bool supported = input.support_force_valid &&
        std::isfinite(input.measured_support_force_n.left) &&
        std::isfinite(input.measured_support_force_n.right) &&
        input.measured_support_force_n.left >= 0.1*config_.robot_mass*kGravity &&
        input.measured_support_force_n.right >= 0.1*config_.robot_mass*kGravity;
    const bool extension_eligible = allow_extensions && !external_support && height.valid &&
        std::abs(height.ground_mismatch_m) <= .015 && supported;
    if (extension_eligible) lqr14_support_elapsed_s_ += input.dt;
    else lqr14_support_elapsed_s_ = 0.;
    const bool height_active = extension_eligible && lqr14_support_elapsed_s_ >= .05;
    if (height_active) lqr14_blend_ = std::min(1., lqr14_blend_+input.dt/.25);
    else lqr14_blend_ = 0.;
    diag.height_control_active = height_active;
    diag.blend = lqr14_blend_;
    const double alpha_ref =
        balance_theta_reference_ + balance_pitch_reference_;
    const double roll_ref = config_.lqr14_roll_sign * balance_roll_reference_;
    const double projection_ref =
        std::sin(balance_pitch_reference_) * std::sin(alpha_ref) +
        std::cos(roll_ref) * std::cos(balance_pitch_reference_) *
            std::cos(alpha_ref);
    diag.height_reference_m =
        config_.wheel_radius + target_leg_length_ * projection_ref;
    if (!height.valid || !height_active) {
        lqr14_height_initialized_ = false;
        lqr14_filtered_height_rate_ = 0.;
    } else if (!lqr14_height_initialized_) {
        lqr14_filtered_height_rate_ = height.rate_m_s;
        lqr14_previous_height_reference_ = diag.height_reference_m;
        lqr14_height_initialized_ = true;
    } else {
        const double weight = 1.0 - std::exp(-kTwoPi * 20.0 * input.dt);
        lqr14_filtered_height_rate_ +=
            weight * (height.rate_m_s - lqr14_filtered_height_rate_);
    }
    diag.height_rate_reference_m_s =
        (diag.height_reference_m - lqr14_previous_height_reference_) / input.dt;
    lqr14_previous_height_reference_ = diag.height_reference_m;
    diag.height_rate_m_s = lqr14_filtered_height_rate_;
    using Lqr = UnifiedLqr14;
    Lqr::State measured = Lqr::State::Zero();
    measured << tracking.position_error, tracking.speed_error,
        tracking.yaw_error, tracking.yaw_rate_error,
        input.theta.left - balance_theta_reference_, input.theta_rate.left,
        input.theta.right - balance_theta_reference_, input.theta_rate.right,
        input.pitch - balance_pitch_reference_, input.pitch_rate, model_roll,
        model_roll_rate, height.height_m, lqr14_filtered_height_rate_;
    Lqr::Reference reference;
    reference.state[Lqr::kRoll] = roll_ref;
    reference.state[Lqr::kHeight] = diag.height_reference_m;
    reference.state[Lqr::kHeightRate] = diag.height_rate_reference_m_s;
    if (evaluation.has_pitch_trim) {
        reference.feedforward[Lqr::kLeftBodyOnLegTorque] =
            reference.feedforward[Lqr::kRightBodyOnLegTorque] =
                evaluation.body_on_leg_feedforward_nm;
    }
    reference.feedforward[Lqr::kLeftVerticalForce] =
        reference.feedforward[Lqr::kRightVerticalForce] =
            0.5 * config_.robot_mass * kGravity;
    if (!supported) {
        measured[Lqr::kYaw] = reference.state[Lqr::kYaw] = 0.;
        measured[Lqr::kYawRate] = reference.state[Lqr::kYawRate] = 0.;
    }
    if (!height_active) {
        // Mask the error contribution, never replace an unavailable height with a real measurement.
        measured[Lqr::kRoll] = reference.state[Lqr::kRoll] = 0.;
        measured[Lqr::kRollRate] = reference.state[Lqr::kRollRate] = 0.;
        measured[Lqr::kHeight] = reference.state[Lqr::kHeight] = 0.;
        measured[Lqr::kHeightRate] = reference.state[Lqr::kHeightRate] = 0.;
        reference.feedforward[Lqr::kLeftVerticalForce] = 0.;
        reference.feedforward[Lqr::kRightVerticalForce] = 0.;
    }
    Lqr::Limits limits;
    limits.lower << -wheel_torque_limit_, -wheel_torque_limit_,
        -config_.joint_torque_limit, -config_.joint_torque_limit, 0.0, 0.0;
    limits.upper << wheel_torque_limit_, wheel_torque_limit_,
        config_.joint_torque_limit, config_.joint_torque_limit,
        config_.force_limit, config_.force_limit;
    const Lqr controller(evaluation.design, limits);
    const auto result = controller.Compute(measured, reference);
    if (!result.valid) {
        diag.status = Lqr14Status::kNumericalFailure;
        return false;
    }
    for (int j = 0; j < 14; ++j) {
        diag.left_wheel_contribution[j] =
            -evaluation.design.gain(Lqr::kLeftWheelTorque, j) * result.error[j];
        diag.right_wheel_contribution[j] =
            -evaluation.design.gain(Lqr::kRightWheelTorque, j) * result.error[j];
    }
    diag.requested_wheel_torque = {result.requested[Lqr::kLeftWheelTorque],
                                   result.requested[Lqr::kRightWheelTorque]};
    AllocateBalanceTorques(result.requested[Lqr::kLeftWheelTorque],
                           result.requested[Lqr::kRightWheelTorque],
                           result.requested[Lqr::kLeftBodyOnLegTorque],
                           result.requested[Lqr::kRightBodyOnLegTorque],
                           candidate);
    candidate.roll_compensation_force = 0.;
    ComputeLegOutput(input, Side::kLeft, tracking.target_leg_length_rate, external_support, candidate);
    ComputeLegOutput(input, Side::kRight, tracking.target_leg_length_rate, external_support, candidate);
    const auto length_joint_torque = candidate.joint_torque;
    const auto length_axial_force = candidate.leg_axial_force;
    for (Side side : {Side::kLeft, Side::kRight}) {
        candidate.physical_wheel_torque.Get(side) =
            std::clamp(candidate.physical_wheel_torque.Get(side),
                       -wheel_torque_limit_, wheel_torque_limit_);
        if (input.support_force_valid && std::isfinite(input.measured_support_force_n.Get(side)) &&
            input.measured_support_force_n.Get(side) < .1*config_.robot_mass*kGravity) {
            candidate.physical_wheel_torque.Get(side) = 0.;
        }
        diag.allocated_wheel_torque.Get(side) = candidate.physical_wheel_torque.Get(side);
        if (!height_active) continue;
        const int force_index = side == Side::kLeft ? Lqr::kLeftVerticalForce
                                                    : Lqr::kRightVerticalForce;
        const double vertical_force = result.command[force_index];
        diag.requested_vertical_force_n.Get(side) = vertical_force;
        const auto mapping = MapVerticalForceToAxial(
            input, side, vertical_force, candidate.body_on_leg_torque.Get(side),
            config_);
        if (!mapping.valid) {
            // Geometric allocation failure: keep bounded length control, not zero support.
            side_state_ = saved_side_state;
            return ComputeLqr14Output(input, tracking, candidate, external_support, false);
        }
        diag.allocated_vertical_force_n.Get(side) =
            mapping.realized_vertical_force_n;
        const auto &leg = input.leg.Get(side);
        const auto vmc = ComputeLegVmc(
            leg, leg.length, mapping.axial_force_n, 0.0,
            config_.leg_hardware(side == Side::kRight).vmc_torque_sign *
                candidate.body_on_leg_torque.Get(side),
            0.0, 0.0, config_);
        if (!std::isfinite(vmc.phi1_torque) ||
            !std::isfinite(vmc.phi2_torque)) {
            diag.status = Lqr14Status::kNumericalFailure;
            return false;
        }
        candidate.leg_axial_force.Get(side) = vmc.axial_force;
        candidate.joint_torque.Get(side).b =
            std::clamp(vmc.phi2_torque, -config_.joint_torque_limit,
                       config_.joint_torque_limit);
        candidate.joint_torque.Get(side).d =
            std::clamp(vmc.phi1_torque, -config_.joint_torque_limit,
                       config_.joint_torque_limit);
    }
    if (height_active) {
        for (Side side : {Side::kLeft, Side::kRight}) {
            auto& torque = candidate.joint_torque.Get(side);
            for (Joint joint : {Joint::kB, Joint::kD}) {
                const double base = length_joint_torque.Get(side).Get(joint);
                torque.Get(joint) = base + lqr14_blend_*(torque.Get(joint)-base);
            }
            candidate.leg_axial_force.Get(side) = length_axial_force.Get(side) +
                lqr14_blend_*(candidate.leg_axial_force.Get(side)-length_axial_force.Get(side));
            if (lqr14_blend_ >= 1.) side_state_.Get(side).leg_length_integral_force = 0.;
        }
    }
    candidate.roll_compensation_force = external_support
        ? .5*(external_support->left_force_n-external_support->right_force_n)
        : .5*(diag.requested_vertical_force_n.left-diag.requested_vertical_force_n.right);
    diag.status =
        external_support ? Lqr14Status::kExternalSupport :
        !allow_extensions ? Lqr14Status::kAction :
        !height_active ? Lqr14Status::kDegraded :
        Lqr14Status::kActive;
    return true;
}

void BalanceController::ComputeLegOutput(
    const ChassisCycleInput &input, Side side, double target_leg_length_rate,
    const ExternalSupportForceCommand *external_support,
    ChassisControlOutput &output) {
    const double cos_theta = std::cos(input.theta.Get(side));
    const LegHardwareMap &hardware = config_.leg_hardware(side == Side::kRight);
    const LegKinematics &leg = input.leg.Get(side);
    SideState &state = side_state_.Get(side);
    const bool external_force_active = external_support != nullptr;
    const double length_error = target_leg_length_ - leg.length;
    if (external_force_active) {
        // 外部 MPC 负责高度和支撑力，此处禁用本地腿长环积分，
        // 后续同时抑制 P、D 输入，避免对抗 MPC 的差动力瞬态。
        state.leg_length_integral_force = 0.0;
    } else {
        state.leg_length_integral_force = UpdateLegLengthIntegral(
            state.leg_length_integral_force, length_error, input.dt, config_);
    }
    double support_share = .5;
    if (input.support_force_valid &&
        std::isfinite(input.measured_support_force_n.left) && std::isfinite(input.measured_support_force_n.right)) {
        const double threshold = .1*config_.robot_mass*kGravity;
        const bool left_contact = input.measured_support_force_n.left >= threshold;
        const bool right_contact = input.measured_support_force_n.right >= threshold;
        if (!left_contact || !right_contact) {
            support_share = side == Side::kLeft ? (left_contact ? 1. : 0.) : (right_contact ? 1. : 0.);
        }
    }
    const double nominal_support_force =
        support_share * config_.robot_mass * kGravity / std::max(cos_theta, 0.5);
    const double roll_force = side == Side::kLeft
                                  ? output.roll_compensation_force
                                  : -output.roll_compensation_force;
    double external_vertical_force = 0.0;
    if (external_force_active) {
        external_vertical_force = side == Side::kLeft
                                      ? external_support->left_force_n
                                      : external_support->right_force_n;
    }
    // MPC 和力估计器使用竖直地面反力，
    // 此处转换为 VMC 映射所需的腿部轴向力。
    const double support_force =
        external_force_active
            ? external_vertical_force / std::max(cos_theta, 0.5)
            : nominal_support_force + roll_force;
    // 安装映射负责把“机体对腿”力矩转换成各侧 VMC 输入方向。
    const double vmc_angle_torque =
        hardware.vmc_torque_sign * output.body_on_leg_torque.Get(side);
    const double vmc_target_length =
        external_force_active ? leg.length : target_leg_length_;
    const double vmc_target_length_rate =
        external_force_active ? 0.0 : target_leg_length_rate;
    const double vmc_leg_speed = external_force_active ? 0.0 : leg.length_rate;
    const LegVmcOutput vmc = ComputeLegVmc(
        leg, vmc_target_length, support_force, state.leg_length_integral_force,
        vmc_angle_torque, vmc_leg_speed, vmc_target_length_rate, config_);
    output.leg_axial_force.Get(side) = vmc.axial_force;
    JointPair<double> &joint_torque = output.joint_torque.Get(side);
    joint_torque.b =
        std::isfinite(vmc.phi2_torque)
            ? std::clamp(vmc.phi2_torque, -config_.joint_torque_limit,
                         config_.joint_torque_limit)
            : 0.0;
    joint_torque.d =
        std::isfinite(vmc.phi1_torque)
            ? std::clamp(vmc.phi1_torque, -config_.joint_torque_limit,
                         config_.joint_torque_limit)
            : 0.0;
}

}  // namespace dust_control::chassis
