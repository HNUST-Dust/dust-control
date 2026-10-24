/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include "chassis_config.h"
#include "gas_spring.h"
#include "leg_joint_coordinates.h"

namespace modules {

// 共享几何参数映射；结果为 [B, D] 顺序的电机反向补偿力矩，
// 使用原始运动学坐标，不使用估计器的传感器极性。
inline JointPair<double> ConfiguredGasSpringCompensation(
    Side side, const ChassisCycleInput &input) {
    using namespace chassis_config;
    const double force =
        side == Side::kLeft ? kGasSpringLeftForce : kGasSpringRightForce;
    const double offset = side == Side::kLeft ? kGasSpringLeftUpperOffset
                                              : kGasSpringRightUpperOffset;
    const auto torque = ComputeGasSpringCompensation(
        input.leg.Get(side), JointDLinkAngle(input.joint_position.Get(side).d),
        kLengthAG, kLengthGH, offset, kGasSpringLowerDistance, force);
    return {torque[1], torque[0]};
}

}  // namespace modules
