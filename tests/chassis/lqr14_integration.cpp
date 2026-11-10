/* SPDX-License-Identifier: Apache-2.0 */
#include <cassert>
#include <cmath>

#include "chassis/balance_controller.h"
#include "chassis/balance_lqr14.h"
#include "chassis/leg_joint_coordinates.h"
#include "chassis/leg_vmc.h"

namespace c = dust_control::chassis;
namespace {
c::ChassisConfig Config(bool shadow = false) {
    c::ChassisConfig config;
    config.lqr14_enabled = true;
    config.lqr14_shadow_only = shadow;
    config.min_commanded_leg_length = .16;
    config.max_commanded_leg_length = .30;
    config.default_leg_length = .20;
    config.theta_balance_bias_deg = 0;
    return config;
}
c::ChassisCycleInput Pose(const c::ChassisConfig& config) {
    c::ChassisCycleInput input;
    input.feedback_valid = input.imu_fresh = input.requested_enable = true;
    input.support_force_valid = true;
    input.measured_support_force_n = {60, 60};
    input.dt = .001;
    for (auto side : {c::Side::kLeft, c::Side::kRight}) {
        const auto hardware = config.leg_hardware(side == c::Side::kRight);
        const double raw = c::RawLegAngle(0, hardware);
        double d = 0, b = 0;
        bool solved = false;
        for (int sd = -3; sd <= 3 && !solved; ++sd) {
            for (int sb = -3; sb <= 3 && !solved; ++sb) {
                solved = c::InverseKinematics(
                    .20 * std::sin(raw), -.20 * std::cos(raw),
                    hardware.kinematic_branch, sd, sb, d, b, config);
            }
        }
        assert(solved);
        auto& leg = input.leg.Get(side);
        assert(c::ComputeLegKinematics(d, b, 0, 0, hardware.kinematic_branch,
                                       leg, config));
        leg.angle = c::CalibratedLegAngle(leg.angle, hardware);
        leg.angle_rate *= hardware.leg_coordinate_sign;
        input.theta.Get(side) = leg.angle;
    }
    return input;
}
c::ChassisControlOutput Warm(c::BalanceController& controller,
                             const c::ChassisCycleInput& input,
                             int count = 400) {
    c::ChassisControlOutput out;
    for (int i = 0; i < count; ++i) {
        out = {};
        controller.Update(input, out);
    }
    return out;
}
void Near(double a, double b, double epsilon = 1e-7) {
    assert(std::abs(a - b) < epsilon);
}
void SameActuators(const c::ChassisControlOutput& a,
                   const c::ChassisControlOutput& b) {
    for (auto side : {c::Side::kLeft, c::Side::kRight}) {
        Near(a.physical_wheel_torque.Get(side),
             b.physical_wheel_torque.Get(side));
        Near(a.joint_torque.Get(side).b, b.joint_torque.Get(side).b);
        Near(a.joint_torque.Get(side).d, b.joint_torque.Get(side).d);
    }
}
void HeightAndForce() {
    const auto config = Config();
    auto input = Pose(config);
    auto h = c::EstimateFlatGroundHipHeight(input, config);
    assert(h.valid);
    Near(h.height_m, .258, 3e-5);
    Near(h.ground_mismatch_m, 0, 3e-5);
    // 原始右腿 hz 与左腿镜像；校准后的高度必须一致。
    Near(h.vertical_projection.left, h.vertical_projection.right, 3e-5);
    for (auto side : {c::Side::kLeft, c::Side::kRight}) {
        auto& leg = input.leg.Get(side);
        leg.length = .22;
        leg.angle = .12;
        leg.angle_rate = -.2;
        leg.length_rate = .03;
        input.theta.Get(side) = .12;
    }
    input.pitch = .04;
    input.pitch_rate = .1;
    input.roll = .05;
    input.roll_rate = -.08;
    h = c::EstimateFlatGroundHipHeight(input, config);
    const double epsilon = 1e-6;
    auto plus = input, minus = input;
    plus.pitch += epsilon * input.pitch_rate;
    minus.pitch -= epsilon * input.pitch_rate;
    plus.roll += epsilon * input.roll_rate;
    minus.roll -= epsilon * input.roll_rate;
    for (auto side : {c::Side::kLeft, c::Side::kRight}) {
        plus.leg.Get(side).length += epsilon * input.leg.Get(side).length_rate;
        minus.leg.Get(side).length -= epsilon * input.leg.Get(side).length_rate;
        plus.leg.Get(side).angle += epsilon * input.leg.Get(side).angle_rate;
        minus.leg.Get(side).angle -= epsilon * input.leg.Get(side).angle_rate;
    }
    Near(h.rate_m_s, (c::EstimateFlatGroundHipHeight(plus, config).height_m -
                      c::EstimateFlatGroundHipHeight(minus, config).height_m) /
                         (2 * epsilon));
    for (auto side : {c::Side::kLeft, c::Side::kRight}) {
        const auto map =
            c::MapVerticalForceToAxial(input, side, 55, 1.2, config);
        assert(map.valid);
        Near(map.realized_vertical_force_n, 55);
    }
    input = Pose(config);
    const auto map =
        c::MapVerticalForceToAxial(input, c::Side::kLeft, 60, 0, config);
    assert(map.valid);
    Near(map.axial_force_n, 60 - .6 * c::chassis_config::kGravity, 1e-5);
    input.feedback_valid = false;
    assert(!c::EstimateFlatGroundHipHeight(input, config).valid);
}
void Integration() {
    auto config = Config();
    const auto input = Pose(config);
    c::BalanceController controller(4.7, config);
    controller.ResetForEnable();
    auto out = Warm(controller, input);
    assert(out.lqr14.status == c::Lqr14Status::kActive);
    Near(out.lqr14.blend, 1);
    Near(out.lqr14.requested_vertical_force_n.left, 12.054 * 9.80665 / 2, .05);
    // 在完整接管后，旧腿长 PID 参数不得改变新输出。
    auto other_config = config;
    other_config.leg_length_kp = 1e7;
    other_config.leg_length_kd = 1e7;
    other_config.leg_length_ki = 1e7;
    c::BalanceController other(4.7, other_config);
    other.ResetForEnable();
    SameActuators(out, Warm(other, input));
    auto moving = input;
    moving.imu.yaw_rate_rad_s = 2.0;
    out = Warm(controller, moving);
    assert(out.lqr14.status == c::Lqr14Status::kActive);
    auto unloaded = input;
    unloaded.measured_support_force_n.left = 0;
    out = Warm(controller, unloaded, 1);
    assert(out.lqr14.status == c::Lqr14Status::kWaitingSupport);
    Near(out.lqr14.blend, 0);
    out = Warm(controller, input, 10);
    assert(out.lqr14.status == c::Lqr14Status::kWaitingSupport);
    Warm(controller, input);
    auto sloped = input;
    sloped.leg.left.length += .05;
    out = Warm(controller, sloped, 1);
    assert(out.lqr14.status == c::Lqr14Status::kOutsideModel);
    c::ExternalSupportForceCommand external;
    external.left_force_n = 40;
    external.right_force_n = 50;
    controller.Update(input, out, &external);
    assert(out.lqr14.status == c::Lqr14Status::kExternalSupport);
    controller.Update(input, out, nullptr, false);
    assert(out.lqr14.status == c::Lqr14Status::kAction);
    controller.Reset();
    out = Warm(controller, input, 1);
    assert(out.lqr14.status == c::Lqr14Status::kWaitingSupport);
    auto wrong_dt = input;
    wrong_dt.dt = .01;
    out = Warm(controller, wrong_dt, 1);
    assert(out.lqr14.status == c::Lqr14Status::kInvalidFeedback);
}
void Shadow() {
    auto config = Config(true);
    auto legacy_config = config;
    legacy_config.lqr14_enabled = false;
    const auto input = Pose(config);
    c::BalanceController shadow(4.7, config), legacy(4.7, legacy_config);
    shadow.ResetForEnable();
    legacy.ResetForEnable();
    const auto a = Warm(shadow, input);
    const auto b = Warm(legacy, input);
    assert(a.lqr14.status == c::Lqr14Status::kShadow);
    Near(a.lqr14.blend, 0);
    SameActuators(a, b);
}
}  // namespace
int main() {
    Eigen::internal::set_is_malloc_allowed(false);
    HeightAndForce();
    Integration();
    Shadow();
}
