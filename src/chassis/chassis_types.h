/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <chassis/controller_inputs.h>

#include <cstdint>
#include <hardware/motors/motor_targets.hpp>

#include "leg_kinematics.h"

namespace modules {

enum class Side : uint8_t {
    kLeft = 0U,
    kRight,
};

enum class Joint : uint8_t {
    kB = 0U,
    kD,
};

template <typename T>
struct SidePair {
    T left{};
    T right{};

    T &Get(Side side) {
        return side == Side::kLeft ? left : right;
    }

    const T &Get(Side side) const {
        return side == Side::kLeft ? left : right;
    }
};

template <typename T>
struct JointPair {
    T b{};
    T d{};

    T &Get(Joint joint) {
        return joint == Joint::kB ? b : d;
    }

    const T &Get(Joint joint) const {
        return joint == Joint::kB ? b : d;
    }
};

enum class ChassisControlState : uint8_t {
    kDisabled = 0U,
    kSafetyStop,
    kDmArming,
    kWaitingFeedback,
    kRecovery,
    kBalance,
    kFlight,
    kJump,
    kClimbStairs,
    kActionFault,
    kTiltFault,
};

enum class DmArmResetReason : uint8_t {
    kStartup = 0U,
    kRemoteEnableEdge,
    kImuFreshness,
    kTiltFault,
};

/** 本控制周期冻结的输入、估计值与有效性状态。 */
struct ChassisCycleInput {
    dust::devices::ImuSample imu;
    msg::RemoteInputState remote = {};
    SidePair<LegKinematics> leg;
    SidePair<double> wheel_motor_speed_rpm;
    SidePair<JointPair<double>> joint_position;
    SidePair<JointPair<double>> joint_velocity;
    SidePair<JointPair<double>> joint_torque_feedback;
    double dt = 0.001;
    double pitch = 0.0;
    double pitch_rate = 0.0;
    double roll = 0.0;
    double roll_rate = 0.0;
    SidePair<double> theta;
    SidePair<double> theta_rate;
    double common_theta = 0.0;
    double common_theta_rate = 0.0;
    bool imu_fresh = false;
    uint64_t imu_age_us = UINT64_MAX;
    uint32_t remote_age_ms = UINT32_MAX;
    bool remote_fresh = false;
    bool feedback_valid = false;
    uint8_t feedback_invalid_mask = 0U;  ///< 安全位：电机超时、IMU 超时、
                                         ///< 左/右运动学无效。
    uint8_t motor_stale_mask =
        0U;  ///< 位顺序与 dust::devices::ActuatorId 一致。
    uint32_t motor_feedback_age_us[6] = {};  ///< 顺序与 motor_stale_mask 一致。
    bool requested_enable = false;
    bool dm_ready = false;
    bool arm_complete = false;
};

/** 本周期生成的执行器目标与估计状态，物理量单位由字段注明。 */
struct ChassisControlOutput {
    dust::devices::MotorTargets motor_targets{};
    // 必须显式启用；默认或零输出不能触发弹簧补偿。
    bool joint_control_active = false;
    // 仅开环跳跃推力阶段绕过反向弹簧补偿。
    bool bypass_gas_spring_compensation = false;
    SidePair<double> physical_wheel_torque;
    SidePair<double> requested_body_on_leg_torque;
    SidePair<double> body_on_leg_torque;
    SidePair<double> leg_axial_force;
    SidePair<JointPair<double>> joint_torque;
    double raw_body_speed = 0.0;
    double estimated_body_speed = 0.0;
    double measured_turn_wheel_speed = 0.0;
    double requested_turn_torque = 0.0;
    double allocated_turn_torque = 0.0;
    double differential_leg_torque = 0.0;
    double roll_reference = 0.0;
    double roll_compensation_force = 0.0;
    SidePair<double> external_support_force_target;
    SidePair<double> support_force_feedback_correction;
    bool support_force_feedback_applied = false;
};

struct ChassisSupportForceEstimate {
    SidePair<double> raw_force_n;
    SidePair<double> filtered_force_n;
    SidePair<double> leg_vertical_force_n;
    SidePair<double> wheel_vertical_accel_mps2;
    double body_vertical_accel_mps2 = 0.0;
    double world_vertical_specific_force_g = 0.0;
    bool valid = false;
};

/** 外部控制器提供的左右绝对垂直地面反力目标。 */
struct ExternalSupportForceCommand {
    uint32_t sequence = 0U;
    double left_force_n = 0.0;
    double right_force_n = 0.0;
};

}  // namespace modules
