/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <motors_params_generated.h>

#include <array>
#include <cstdint>

namespace platform::motor_map {

/** 执行器命令和反馈路由共用的 CAN 地址。 */
struct MotorAddress {
    uint8_t bus;
    uint16_t command_id;
    uint16_t feedback_id;
    uint8_t command_slot;  // DJI 组帧索引，DM MIT 关节不使用。
};

inline constexpr MotorAddress kLeftWheel = {
    motors_params::kLeftWheelBus, motors_params::kLeftWheelCommandId,
    motors_params::kLeftWheelFeedbackId, motors_params::kLeftWheelCommandSlot};
inline constexpr MotorAddress kRightWheel = {
    motors_params::kRightWheelBus, motors_params::kRightWheelCommandId,
    motors_params::kRightWheelFeedbackId,
    motors_params::kRightWheelCommandSlot};
inline constexpr MotorAddress kLeftJointB = {
    motors_params::kLeftJointBBus, motors_params::kLeftJointBCommandId,
    motors_params::kLeftJointBFeedbackId, 0U};
inline constexpr MotorAddress kLeftJointD = {
    motors_params::kLeftJointDBus, motors_params::kLeftJointDCommandId,
    motors_params::kLeftJointDFeedbackId, 0U};
inline constexpr MotorAddress kRightJointB = {
    motors_params::kRightJointBBus, motors_params::kRightJointBCommandId,
    motors_params::kRightJointBFeedbackId, 0U};
inline constexpr MotorAddress kRightJointD = {
    motors_params::kRightJointDBus, motors_params::kRightJointDCommandId,
    motors_params::kRightJointDFeedbackId, 0U};

inline constexpr std::array<MotorAddress, 6> kDefaultMotors = {
    kLeftWheel,  kRightWheel,  kLeftJointB,
    kLeftJointD, kRightJointB, kRightJointD};

}  // namespace platform::motor_map
