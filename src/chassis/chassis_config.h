/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace modules::chassis_config {
// 多个底盘模块共用的物理量、坐标约定及关节控制限值。
inline constexpr double kGravity = 9.80665;
inline constexpr double kLegLengthCommandRate = 0.24;
inline constexpr double kYawTrackingErrorLimitDeg = 45.0;
inline constexpr double kJointPositionKi = 0.0;
inline constexpr double kJointSpeedKi = 0.0;
inline constexpr double kJointIntegralTorqueLimit = 20.0;
inline constexpr double kJointTorqueMax = 54.0;
inline constexpr int kLeftKinematicBranch = 1;
inline constexpr double kLeftLegCoordinateSign = -1.0;
inline constexpr double kLeftWheelFeedbackSign = 1.0;
inline constexpr double kLeftVmcTorqueSign = -1.0;
inline constexpr int kRightKinematicBranch = -1;
inline constexpr double kRightLegCoordinateSign = 1.0;
inline constexpr double kRightWheelFeedbackSign = -1.0;
inline constexpr double kRightVmcTorqueSign = 1.0;

inline constexpr uint64_t kControlPeriodNs = 1000000;
inline constexpr double kDefaultDt =
    static_cast<double>(kControlPeriodNs) / 1e9;

struct LegHardwareMap {
    int kinematic_branch;
    double leg_angle_offset;
    double leg_coordinate_sign;
    double wheel_feedback_sign;
    double vmc_torque_sign;
};

inline constexpr double kTwoPi = 6.28318530717958647692;
inline constexpr double kDegToRad = 0.01745329251994329577;
inline constexpr double kDpsToRadPerSec = kDegToRad;
inline constexpr double kRpmToRadPerSec = 0.10471975511965977;
inline constexpr double kYawTrackingErrorLimit =
    kYawTrackingErrorLimitDeg * kDegToRad;
// 离线控制器的默认轮力矩预算；正式运行由设备层注入实际协议限值。
inline constexpr double kWheelTorqueLimit = 4.748985507246377;

}  // namespace modules::chassis_config

namespace modules {
// 启动时加载后按值复制给使用者，控制周期不读 YAML、不访问全局可变配置。
struct ChassisConfig {
    double joint_b_zero_offset = 0.0;
    double joint_d_zero_offset = 3.141592653589793;
    double wheel_reduction = 15.764705882352942;
    double wheel_radius = 0.058;
    double half_wheel_track = 0.215;
    double robot_mass = 12.054;
    double theta_balance_bias_deg = 5.0;
    double remote_deadband = 0.05;
    double min_commanded_leg_length = 0.15;
    double max_commanded_leg_length = 0.33;
    double max_body_speed = 1.6;
    double body_acceleration_limit = 0.8;
    double max_turn_wheel_speed = 2.7;
    double turn_acceleration_limit = 1.5;
    double turn_torque_limit = 1.2;
    double roll_compensation_kp = 80.0;
    double roll_compensation_force_limit = 20.0;
    double joint_torque_limit = 54.0;
    double tilt_cutoff_deg = 75.0;
    double length_ab = 0.0945;
    double length_bc = 0.1125;
    double length_cd = 0.116;
    double length_ad = 0.09;
    double length_ag = 0.21;
    double length_gh = 0.25;
    double stool_target_leg_length = 0.15;
    double joint_position_kp = 20.0;
    double joint_position_max_speed = 44.0;
    double joint_speed_kp = 4.0;
    double gas_spring_left_force = 300.0;
    double gas_spring_right_force = 300.0;
    double gas_spring_left_upper_offset = 0.05;
    double gas_spring_right_upper_offset = -0.05;
    double gas_spring_lower_distance = 0.05;
    double support_wheel_mass = 0.6;
    double flight_airborne_push_force = 80.0;
    double jump_thrust_torque = 30.0;
    double leg_length_kp = 600.0;
    double leg_length_ki = 100.0;
    double leg_length_kd = 120.0;
    double integral_force_limit = 60.0;
    double force_limit = 200.0;
    double left_leg_angle_offset = -0.036063;
    double right_leg_angle_offset = -3.12101;
    double max_yaw_rate() const {
        return max_turn_wheel_speed / half_wheel_track;
    }
    double yaw_acceleration_limit() const {
        return turn_acceleration_limit / half_wheel_track;
    }
    double theta_balance_bias() const {
        return theta_balance_bias_deg * chassis_config::kDegToRad;
    }
    chassis_config::LegHardwareMap leg_hardware(bool right) const {
        using namespace chassis_config;
        return right
                   ? LegHardwareMap{kRightKinematicBranch,
                                    right_leg_angle_offset,
                                    kRightLegCoordinateSign,
                                    kRightWheelFeedbackSign,
                                    kRightVmcTorqueSign}
                   : LegHardwareMap{kLeftKinematicBranch, left_leg_angle_offset,
                                    kLeftLegCoordinateSign,
                                    kLeftWheelFeedbackSign, kLeftVmcTorqueSign};
    }
};
void ValidateChassisConfig(const ChassisConfig& config);
ChassisConfig LoadChassisConfig(const std::string& file);
}  // namespace modules
