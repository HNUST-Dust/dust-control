/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include "chassis_config.h"

namespace dust_control::chassis {

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
