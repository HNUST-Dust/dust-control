/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file src/chassis/leg_vmc.h
 * @ingroup wbr_modules
 * @brief 实现腿部虚拟模型控制的力矩映射。
 * @details 由控制线程同步调用，不创建线程或执行设备 I/O。
 */

#pragma once

#include "chassis_config.h"
#include "leg_kinematics.h"

namespace dust_control::chassis {

/** @brief 腿部虚拟力映射得到的关节力矩。 */
struct LegVmcOutput {
    double axial_force = 0.0;  ///< 沿腿轴方向的控制力，单位为牛。
    double phi1_torque = 0.0;  ///< D 电机坐标力矩，单位为牛·米。
    double phi2_torque = 0.0;  ///< B 电机坐标力矩，单位为牛·米。
};

/**
 * @brief 更新带限幅的腿长误差积分项。
 * @param integral_force 上一周期累计的轴向力积分项。
 * @param length_error 目标腿长减实际腿长的误差，单位为米。
 * @param dt 相邻两次更新的时间间隔，单位为秒且必须为正。
 * @return 经过积分和限幅后的轴向力积分项。
 */
double UpdateLegLengthIntegral(double integral_force, double length_error,
                               double dt, const ChassisConfig& config = {});
/**
 * @brief 将期望轴向力和髋部力矩映射为关节力矩。
 * @param[in] leg 腿部运动学输入或输出状态。
 * @param target_leg_length 目标腿长，单位为米。
 * @param support_feedforward 抵消机体重力的腿部支撑前馈力，单位为牛。
 * @param integral_force 上一周期累计的轴向力积分项。
 * @param leg_angle_torque 原始运动学腿角的力矩，单位为牛·米。
 * 调用方先将机体对腿力矩乘以对应侧的 vmc_torque_sign。
 * @param filtered_leg_speed 经过滤波的实际腿长变化率，单位为米每秒。
 * @param target_leg_length_rate 目标腿长变化率，单位为米每秒。
 * @return 同一条腿的 D/B 电机力矩，不含弹簧补偿或关节力矩限幅。
 */
LegVmcOutput ComputeLegVmc(const LegKinematics& leg, double target_leg_length,
                           double support_feedforward, double integral_force,
                           double leg_angle_torque, double filtered_leg_speed,
                           double target_leg_length_rate,
                           const ChassisConfig& config = {});

}  // namespace dust_control::chassis
