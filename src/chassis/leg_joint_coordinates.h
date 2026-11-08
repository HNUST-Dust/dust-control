/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <cmath>

#include "chassis_config.h"

namespace dust_control::chassis {

// 原始运动学腿角与统一腿角 alpha 的双向转换，角度单位均为 rad。
// 安装方向为 ±1；只转换角度，不旋转 hx/hz 或雅可比。
inline double CalibratedLegAngle(
    double raw_angle, const chassis_config::LegHardwareMap& hardware) {
    return hardware.leg_coordinate_sign *
           std::remainder(raw_angle - hardware.leg_angle_offset,
                          chassis_config::kTwoPi);
}

inline double RawLegAngle(double calibrated_angle,
                          const chassis_config::LegHardwareMap& hardware) {
    const double raw_angle = hardware.leg_coordinate_sign < 0.0
                                 ? hardware.leg_angle_offset - calibrated_angle
                                 : calibrated_angle + hardware.leg_angle_offset;
    return std::remainder(raw_angle, chassis_config::kTwoPi);
}

// 仅应用常量偏移：d(连杆角)/d(电机角) = 1，
// 因此速度、雅可比列和力矩仍使用相同的电机坐标。
inline double JointBLinkAngle(double motor_angle,
                              const ChassisConfig& config = {}) {
    return motor_angle + config.joint_b_zero_offset;
}

inline double JointDLinkAngle(double motor_angle,
                              const ChassisConfig& config = {}) {
    return motor_angle + config.joint_d_zero_offset;
}

inline double JointBMotorAngle(double link_angle,
                               const ChassisConfig& config = {}) {
    return link_angle - config.joint_b_zero_offset;
}

inline double JointDMotorAngle(double link_angle,
                               const ChassisConfig& config = {}) {
    return link_angle - config.joint_d_zero_offset;
}

}  // namespace dust_control::chassis
