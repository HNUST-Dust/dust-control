/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include "chassis_config.h"
#include "chassis_types.h"

namespace dust_control::chassis {
struct FlatGroundHipHeight {
    bool valid = false;
    double height_m = 0.0;
    double rate_m_s = 0.0;
    double ground_mismatch_m = 0.0;
    SidePair<double> vertical_projection;
};

// 原始 hx/hz 不参与统一高度。使用校准 alpha 和 IMU 姿态投影，
// 假设左右轮接触同一水平面；是否接地由调用方另行判断。
FlatGroundHipHeight EstimateFlatGroundHipHeight(const ChassisCycleInput& input,
                                                const ChassisConfig& config);

struct VerticalForceMapping {
    bool valid = false;
    double axial_force_n = 0.0;
    double realized_vertical_force_n = 0.0;
};

// 竖直地面反力 → VMC 轴向驱动力：扣除轮重，计入姿态切向力的竖直分量。
// 仍是质量集总近似，不包含轮竖直加速度、腿自重形变和侧向力分配。
VerticalForceMapping MapVerticalForceToAxial(const ChassisCycleInput& input,
                                             Side side, double vertical_force_n,
                                             double body_on_leg_torque_nm,
                                             const ChassisConfig& config);
}  // namespace dust_control::chassis
