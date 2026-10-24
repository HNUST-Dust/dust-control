/* SPDX-License-Identifier: Apache-2.0 */

#include "chassis_input_reader.h"

#include <cmath>

#include "chassis_config.h"
#include "leg_kinematics.h"

namespace modules {
using namespace chassis_config;

void ChassisInputReader::Read(const ControllerInputs &sources,
                              ChassisCycleInput &input, uint64_t now_us,
                              double dt, bool arm_complete) {
    input.dt = dt;

    const auto &motors = sources.motors;
    const SidePair<JointPair<dust::devices::MotorFeedback>> joints = {
        {motors[2], motors[3]}, {motors[4], motors[5]}};

    // sequence 用于区分新样本和通道中残留的旧值。
    input.remote = {};
    input.remote = sources.remote;
    if (input.remote.sequence != 0U &&
        input.remote.sequence != last_remote_sequence_) {
        last_remote_sequence_ = input.remote.sequence;
        last_remote_update_us_ = now_us;
    }

    input.remote_age_ms =
        last_remote_sequence_ != 0U
            ? static_cast<uint32_t>((now_us - last_remote_update_us_) / 1000)
            : UINT32_MAX;
    input.remote_fresh =
        last_remote_sequence_ != 0U && now_us >= last_remote_update_us_ &&
        now_us - last_remote_update_us_ <= deadlines_.remote_us;
    input.requested_enable = input.remote_fresh && input.remote.robot_enable;

    dust::devices::ImuSample new_imu = {};
    new_imu = sources.imu;
    if (new_imu.sequence != 0U) {
        input.imu = new_imu;
        if (input.imu.sequence != last_imu_sequence_) {
            last_imu_sequence_ = input.imu.sequence;
        }
    }

    // 本周期使用调用方提供的单调时间，所有设备年龄基于同一份冻结快照计算。
    const uint64_t feedback_now_us = now_us;
    input.imu_age_us =
        input.imu.valid && input.imu.precise_timestamp_us <= feedback_now_us
            ? feedback_now_us - input.imu.precise_timestamp_us
            : UINT64_MAX;
    input.imu_fresh = input.imu.valid && last_imu_sequence_ != 0U &&
                      input.imu_age_us <= deadlines_.hi91_us;

    const auto frame_age_us = [feedback_now_us](
                                  const dust::devices::MotorFeedback &frame) {
        if (!frame.valid || frame.precise_timestamp_us > feedback_now_us) {
            return UINT32_MAX;
        }
        const uint64_t age_us = feedback_now_us - frame.precise_timestamp_us;
        return age_us > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(age_us);
    };
    for (unsigned i = 0; i < motors.size(); ++i) {
        input.motor_feedback_age_us[i] = frame_age_us(motors[i]);
    }
    const bool motor_fresh[] = {
        input.motor_feedback_age_us[0] <= deadlines_.motors[0],
        input.motor_feedback_age_us[1] <= deadlines_.motors[1],
        input.motor_feedback_age_us[2] <= deadlines_.motors[2],
        input.motor_feedback_age_us[3] <= deadlines_.motors[3],
        input.motor_feedback_age_us[4] <= deadlines_.motors[4],
        input.motor_feedback_age_us[5] <= deadlines_.motors[5],
    };

    input.motor_stale_mask = 0U;
    bool motors_fresh = true;
    for (uint8_t i = 0U; i < sizeof(motor_fresh) / sizeof(motor_fresh[0]);
         ++i) {
        if (!motor_fresh[i]) {
            input.motor_stale_mask |= static_cast<uint8_t>((1U << i));
            motors_fresh = false;
        }
    }

    // 正运动学输出统一采用校准后的左右腿坐标。
    input.leg = {};
    SidePair<bool> leg_valid;
    const auto update_leg_kinematics = [&](Side side) {
        const LegHardwareMap &hardware =
            config_.leg_hardware(side == Side::kRight);
        const auto &feedback = joints.Get(side);
        LegKinematics &leg = input.leg.Get(side);
        bool &valid = leg_valid.Get(side);
        if (!feedback.b.valid || !feedback.d.valid) {
            return;
        }
        valid = ComputeLegKinematics(
            feedback.d.position_rad, feedback.b.position_rad,
            feedback.d.velocity_rad_s, feedback.b.velocity_rad_s,
            hardware.kinematic_branch, leg, config_);
        if (valid) {
            leg.angle =
                hardware.leg_coordinate_sign *
                std::remainder(leg.angle - hardware.leg_angle_offset, kTwoPi);
            leg.angle_rate *= hardware.leg_coordinate_sign;
        }
    };

    update_leg_kinematics(Side::kLeft);
    update_leg_kinematics(Side::kRight);

    input.feedback_invalid_mask = static_cast<uint8_t>(
        (input.motor_stale_mask ? 1U : 0U) | (!input.imu_fresh ? 2U : 0U) |
        (!leg_valid.left ? 4U : 0U) | (!leg_valid.right ? 8U : 0U));
    input.feedback_valid =
        motors_fresh && input.imu_fresh && leg_valid.left && leg_valid.right;
    input.wheel_motor_speed_rpm.left = motors[0].speed_rpm;
    input.wheel_motor_speed_rpm.right = motors[1].speed_rpm;
    input.dm_ready = motors[2].enabled && motors[3].enabled &&
                     motors[4].enabled && motors[5].enabled;
    input.arm_complete = arm_complete;

    input.pitch = input.imu_fresh
                      ? static_cast<double>(input.imu.pitch_deg) * kDegToRad
                      : 0.0;
    input.pitch_rate =
        input.imu_fresh ? static_cast<double>(input.imu.pitch_rate_rad_s) : 0.0;
    input.roll = input.imu_fresh
                     ? static_cast<double>(input.imu.roll_deg) * kDegToRad
                     : 0.0;
    input.roll_rate =
        input.imu_fresh ? static_cast<double>(input.imu.roll_rate_rad_s) : 0.0;
    input.theta = {};
    input.theta_rate = {};
    input.common_theta = 0.0;
    input.common_theta_rate = 0.0;
    if (input.feedback_valid) {
        // 论文坐标满足 alpha = theta + pitch，因此 theta = alpha - pitch。
        input.theta.left =
            std::remainder(input.leg.left.angle - input.pitch, kTwoPi);
        input.theta.right =
            std::remainder(input.leg.right.angle - input.pitch, kTwoPi);
        input.theta_rate.left = input.leg.left.angle_rate - input.pitch_rate;
        input.theta_rate.right = input.leg.right.angle_rate - input.pitch_rate;
        input.common_theta = std::remainder(
            0.5 * (input.leg.left.angle + input.leg.right.angle) - input.pitch,
            kTwoPi);
        input.common_theta_rate =
            0.5 * (input.leg.left.angle_rate + input.leg.right.angle_rate) -
            input.pitch_rate;
    }

    for (Side side : {Side::kLeft, Side::kRight}) {
        for (Joint joint : {Joint::kB, Joint::kD}) {
            const auto &feedback = joints.Get(side).Get(joint);
            input.joint_position.Get(side).Get(joint) = feedback.position_rad;
            input.joint_velocity.Get(side).Get(joint) = feedback.velocity_rad_s;
            input.joint_torque_feedback.Get(side).Get(joint) =
                feedback.torque_nm;
        }
    }
}

}  // namespace modules
