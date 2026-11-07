/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <array>
#include <cstdint>

namespace dust_control::devices {
enum class ActuatorId : uint8_t {
    kLeftWheel,
    kRightWheel,
    kLeftJointB,
    kLeftJointD,
    kRightJointB,
    kRightJointD
};
enum class MotorMode { kNone, kTorque, kDisable, kClearError, kEnable };
struct MotorTarget {
    MotorMode mode = MotorMode::kNone;
    double torque_nm = 0.0;
};
// 每周期重新生成；每个执行器最多一项操作，kNone 表示本周期不发送。
// 轮力矩使用机体坐标；关节力矩保留现有电机坐标。
using MotorTargets = std::array<MotorTarget, 6>;

// 设备驱动产生的物理反馈；时间戳使用主机单调时钟。
struct MotorFeedback {
    uint64_t precise_timestamp_us = 0;
    bool valid = false;
    double position_rad = 0.0;
    double velocity_rad_s = 0.0;
    double torque_nm = 0.0;
    double speed_rpm = 0.0;
    bool enabled = false;
};

}  // namespace dust_control::devices
