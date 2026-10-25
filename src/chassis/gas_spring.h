/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <array>
#include <cmath>

#include "leg_kinematics.h"

namespace dust_control::modules {

// 上安装点 P = (offset/AG) * (-Gz, Gx)，与 AG 刚性连接。
// offset 的符号选择 AG 的安装侧；Q = G + (b/GH)(H-G)。
// 伸长弹簧产生广义力矩 +F * d|Q-P|/dq。
// 按原始运动学列顺序 [D, B] 返回方向相反的电机补偿力矩。
inline std::array<double, 2> ComputeGasSpringCompensation(
    const LegKinematics &leg, double phi_d, double ag, double gh, double offset,
    double b, double force) {
    if (!std::isfinite(phi_d) || !std::isfinite(ag) || !std::isfinite(gh) ||
        !std::isfinite(offset) || !std::isfinite(b) || !std::isfinite(force) ||
        ag <= 0.0 || gh <= 0.0 || b < 0.0 || b > gh || force <= 0.0) {
        return {};
    }

    const double ratio = b / gh;
    const double coefficient = 1.0 - ratio;
    const double normal_ratio = offset / ag;
    const double gx = ag * std::cos(phi_d);
    const double gz = -ag * std::sin(phi_d);
    const double sx = coefficient * gx + ratio * leg.hx + normal_ratio * gz;
    const double sz = coefficient * gz + ratio * leg.hz - normal_ratio * gx;
    const double length = std::hypot(sx, sz);
    if (!std::isfinite(length) || length < 1.0e-6) {
        return {};
    }

    std::array<double, 2> torque{};
    for (size_t column = 0U; column < 2U; ++column) {
        const double dx =
            ratio * leg.jacobian[0][column] +
            (column == 0U ? coefficient * gz - normal_ratio * gx : 0.0);
        const double dz =
            ratio * leg.jacobian[1][column] -
            (column == 0U ? coefficient * gx + normal_ratio * gz : 0.0);
        torque[column] = -force * (sx * dx + sz * dz) / length;
        if (!std::isfinite(torque[column])) {
            return {};
        }
    }

    return torque;
}

}  // namespace dust_control::modules
