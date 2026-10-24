/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <chassis_params_generated.h>

namespace modules {

// 仅应用常量偏移：d(连杆角)/d(电机角) = 1，
// 因此速度、雅可比列和力矩仍使用相同的电机坐标。
inline double JointBLinkAngle(double motor_angle) {
    return motor_angle + chassis_params::kJointBZeroOffset;
}

inline double JointDLinkAngle(double motor_angle) {
    return motor_angle + chassis_params::kJointDZeroOffset;
}

inline double JointBMotorAngle(double link_angle) {
    return link_angle - chassis_params::kJointBZeroOffset;
}

inline double JointDMotorAngle(double link_angle) {
    return link_angle - chassis_params::kJointDZeroOffset;
}

}  // namespace modules
