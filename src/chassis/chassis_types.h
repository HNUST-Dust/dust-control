/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <chassis/controller_inputs.h>

#include <cstdint>
#include <hardware/motors/motor_types.hpp>

#include "leg_kinematics.h"

namespace dust_control::chassis {

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

enum class Lqr14Status : uint8_t {
    kDisabled = 0,
    kShadow,
    kActive,
    kWaitingSupport,
    kInvalidFeedback,
    kOutsideModel,
    kExternalSupport,
    kAction,
    kNumericalFailure,
};
struct Lqr14Diagnostics {
    Lqr14Status status = Lqr14Status::kDisabled;
    bool height_valid = false;
    double height_m = 0.0;
    double height_rate_m_s = 0.0;
    double height_reference_m = 0.0;
    double height_rate_reference_m_s = 0.0;
    double ground_mismatch_m = 0.0;
    double blend = 0.0;
    // -K(row,j)*error[j]，按14状态顺序；模型请求，未分配或限幅。
    double left_wheel_contribution[14] = {};
    double right_wheel_contribution[14] = {};
    SidePair<double> requested_wheel_torque;
    SidePair<double> allocated_wheel_torque;
    SidePair<double> output_wheel_torque;  // 混合后控制命令，非电机实测。
    SidePair<double> requested_vertical_force_n;
    SidePair<double> allocated_vertical_force_n;
};

/** 本控制周期冻结的输入、估计值与有效性状态。 */
struct ChassisCycleInput {
    // 设备样本保留角度 deg、角速度 rad/s、加速度 g；pitch/roll 在下方另转 rad。
    dust_control::devices::ImuSample imu;
    dust_control::input::RemoteInputState remote = {};
    // 仅 angle/angle_rate 已统一左右方向；hx/hz/jacobian 保留原始几何坐标。
    SidePair<LegKinematics> leg;
    // 电机端原始 rpm；平衡估计时再除减速比并处理安装方向。
    SidePair<double> wheel_motor_speed_rpm;
    SidePair<double> wheel_current_feedback_raw;
    // 电机坐标 rad、rad/s、N·m；不预先叠加 B/D 安装零偏。
    SidePair<JointPair<double>> joint_position;
    SidePair<JointPair<double>> joint_velocity;
    SidePair<JointPair<double>> joint_torque_feedback;
    double dt = 0.001;  // s
    // 机体姿态 rad、角速度 rad/s。
    double pitch = 0.0;
    double pitch_rate = 0.0;
    double roll = 0.0;
    double roll_rate = 0.0;
    // theta = 校准腿角 alpha - pitch，单位 rad；对应角速度为 rad/s。
    SidePair<double> theta;
    SidePair<double> theta_rate;
    double common_theta = 0.0;
    double common_theta_rate = 0.0;
    bool imu_fresh = false;
    uint64_t imu_age_us = UINT64_MAX;
    uint32_t remote_age_ms = UINT32_MAX;
    bool remote_fresh = false;
    bool feedback_valid = false;
    // 由总控支撑力估计器在输入读取之后填入；不代表绝对可靠的接触测量。
    bool hip_height_valid = false;
    double hip_height_m = 0.0;
    double hip_height_rate_m_s = 0.0;
    double flat_ground_mismatch_m = 0.0;
    SidePair<double> hip_height_side_m;
    bool support_force_valid = false;
    SidePair<double> measured_support_force_n;
    uint8_t feedback_invalid_mask = 0U;  ///< 安全位：电机超时、IMU 超时、
                                         ///< 左/右运动学无效。
    uint8_t motor_stale_mask =
        0U;  ///< 位顺序与 dust_control::devices::ActuatorId 一致。
    uint32_t motor_feedback_age_us[6] = {};  ///< 顺序与 motor_stale_mask 一致。
    bool requested_enable = false;
    bool dm_ready = false;
    bool arm_complete = false;
};

// 本周期平衡估计与参考；未执行平衡计算时 valid=false。
struct MotionDiagnostics {
    bool valid = false;
    double raw_speed = 0.0;
    double estimated_speed = 0.0;
    double target_speed = 0.0;
    double estimated_position = 0.0;
    double target_position = 0.0;
    double unclamped_position_error = 0.0;
    double position_error = 0.0;  // 控制使用的限幅后误差 m
    double speed_error = 0.0;
    double pitch_reference = 0.0;
    double theta_reference = 0.0;
    double roll_reference = 0.0;
};

/** 本周期生成的执行器目标与估计状态，物理量单位由字段注明。 */
struct ChassisControlOutput {
    dust_control::devices::MotorTargets motor_targets{};
    // 本周期没有显式命令时可用的目标；由输出策略决定，不重放历史命令。
    dust_control::devices::MotorTargets idle_motor_targets{};
    // 必须显式启用；默认或零输出不能触发弹簧补偿。
    bool joint_control_active = false;
    // 仅开环跳跃推力阶段绕过反向弹簧补偿。
    bool bypass_gas_spring_compensation = false;
    // 机体坐标下的轮力矩 N·m；协议命令方向只在设备编码器应用。
    SidePair<double> physical_wheel_torque;
    // 机体对腿的姿态力矩 N·m；进入 VMC 前应用 vmc_torque_sign。
    SidePair<double> requested_body_on_leg_torque;
    SidePair<double> body_on_leg_torque;
    SidePair<double> leg_axial_force;  // 沿腿轴的力 N，不等同于竖直地面反力。
    // 电机坐标 N·m，具名 [B,D]；输出策略在这里加一次弹簧补偿并最终限幅。
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
    MotionDiagnostics motion;
    Lqr14Diagnostics lqr14;
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

}  // namespace dust_control::chassis
