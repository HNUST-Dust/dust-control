/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <array>
#include <cstdint>

#include "dji_model.hpp"

namespace platform::motor_map {

/** 执行器命令和反馈路由共用的 CAN 地址。 */
struct MotorAddress {
    uint8_t bus;
    uint16_t command_id;
    uint16_t feedback_id;
    uint8_t command_slot;  // DJI 组帧索引，DM MIT 关节不使用。
};

// 未指定运行配置时使用的接线基线；显式 YAML 在启动时覆盖实例映射。
inline constexpr MotorAddress kLeftWheel = {3, motor_model::kDjiCommandId,
                                            motor_model::DjiFeedbackId(0), 0};
inline constexpr MotorAddress kRightWheel = {2, motor_model::kDjiCommandId,
                                             motor_model::DjiFeedbackId(0), 0};
inline constexpr MotorAddress kLeftJointB = {3, 0x000, 0x010, 0};
inline constexpr MotorAddress kLeftJointD = {3, 0x003, 0x013, 0};
inline constexpr MotorAddress kRightJointB = {2, 0x001, 0x011, 0};
inline constexpr MotorAddress kRightJointD = {2, 0x002, 0x012, 0};

inline constexpr std::array<MotorAddress, 6> kDefaultMotors = {
    kLeftWheel,  kRightWheel,  kLeftJointB,
    kLeftJointD, kRightJointB, kRightJointD};

}  // namespace platform::motor_map
