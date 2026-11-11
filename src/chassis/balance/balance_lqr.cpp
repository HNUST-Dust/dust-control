/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis/balance/balance_lqr.h"

#include <algorithm>
#include <cmath>

namespace dust_control::chassis {
namespace {
struct Projection {
    double down;
    double rate;
    double tangent_up;
};
Projection Project(double alpha, double alpha_rate, double pitch,
                   double pitch_rate, double roll, double roll_rate) {
    const double sa = std::sin(alpha), ca = std::cos(alpha);
    const double sp = std::sin(pitch), cp = std::cos(pitch);
    const double sr = std::sin(roll), cr = std::cos(roll);
    return {sp * sa + cr * cp * ca,
            pitch_rate * (cp * sa - cr * sp * ca) +
                alpha_rate * (sp * ca - cr * cp * sa) -
                roll_rate * sr * cp * ca,
            -sp * ca + cr * cp * sa};
}
}  // namespace

FlatGroundHipHeight EstimateFlatGroundHipHeight(const ChassisCycleInput& input,
                                                const ChassisConfig& config) {
    FlatGroundHipHeight result;
    if (!input.feedback_valid || !input.imu_fresh) {
        return result;
    }
    const double roll = config.lqr_roll_sign * input.roll;
    const double roll_rate = config.lqr_roll_sign * input.roll_rate;
    double projection_rate_sum = 0.0;
    for (Side side : {Side::kLeft, Side::kRight}) {
        const auto& leg = input.leg.Get(side);
        if (!std::isfinite(leg.length) || leg.length <= 1e-4 ||
            !std::isfinite(leg.length_rate)) {
            return {};
        }
        const auto p = Project(leg.angle, leg.angle_rate, input.pitch,
                               input.pitch_rate, roll, roll_rate);
        if (!std::isfinite(p.down) || !std::isfinite(p.rate) || p.down < 0.5) {
            return {};
        }
        result.vertical_projection.Get(side) = leg.length * p.down;
        projection_rate_sum += leg.length_rate * p.down + leg.length * p.rate;
    }
    result.height_m =
        config.wheel_radius + 0.5 * (result.vertical_projection.left +
                                     result.vertical_projection.right);
    result.rate_m_s = 0.5 * projection_rate_sum;
    result.ground_mismatch_m =
        result.vertical_projection.left - result.vertical_projection.right -
        2.0 * config.half_wheel_track * std::sin(roll) * std::cos(input.pitch);
    result.valid = std::isfinite(result.height_m) &&
                   std::isfinite(result.rate_m_s) &&
                   std::isfinite(result.ground_mismatch_m);
    return result;
}

VerticalForceMapping MapVerticalForceToAxial(const ChassisCycleInput& input,
                                             Side side, double vertical_force_n,
                                             double body_on_leg_torque_nm,
                                             const ChassisConfig& config) {
    const auto& leg = input.leg.Get(side);
    if (!std::isfinite(leg.length) || leg.length <= 1e-4 ||
        !std::isfinite(vertical_force_n) || vertical_force_n < 0.0 ||
        !std::isfinite(body_on_leg_torque_nm)) {
        return {};
    }
    const auto p = Project(leg.angle, 0.0, input.pitch, 0.0,
                           config.lqr_roll_sign * input.roll, 0.0);
    if (!std::isfinite(p.down) || !std::isfinite(p.tangent_up) ||
        p.down < 0.5) {
        return {};
    }
    const double wheel_weight =
        config.support_wheel_mass * chassis_config::kGravity;
    const double tangential_vertical =
        p.tangent_up * body_on_leg_torque_nm / leg.length;
    const double axial =
        (vertical_force_n - wheel_weight + tangential_vertical) / p.down;
    if (!std::isfinite(axial)) {
        return {};
    }
    VerticalForceMapping result;
    result.axial_force_n =
        std::clamp(axial, -config.force_limit, config.force_limit);
    result.realized_vertical_force_n =
        result.axial_force_n * p.down - tangential_vertical + wheel_weight;
    result.valid = std::isfinite(result.realized_vertical_force_n);
    return result;
}
}  // namespace dust_control::chassis
