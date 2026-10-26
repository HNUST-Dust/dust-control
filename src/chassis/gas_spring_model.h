/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include "chassis_config.h"
#include "chassis_types.h"
#include "gas_spring.h"
#include "leg_joint_coordinates.h"

namespace dust_control::chassis {

// 共享几何参数映射；结果为 [B, D] 顺序的电机反向补偿力矩，
// 使用原始运动学坐标，不使用估计器的传感器极性。
inline JointPair<double> ConfiguredGasSpringCompensation(
    Side side, const ChassisCycleInput& input,
    const ChassisConfig& config = {}) {
    using namespace chassis_config;
    const double force = side == Side::kLeft ? config.gas_spring_left_force
                                             : config.gas_spring_right_force;
    const double offset = side == Side::kLeft
                              ? config.gas_spring_left_upper_offset
                              : config.gas_spring_right_upper_offset;
    const auto torque = ComputeGasSpringCompensation(
        input.leg.Get(side),
        JointDLinkAngle(input.joint_position.Get(side).d, config),
        config.length_ag, config.length_gh, offset,
        config.gas_spring_lower_distance, force);
    return {torque[1], torque[0]};
}

}  // namespace dust_control::chassis
