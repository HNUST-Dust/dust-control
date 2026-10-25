/* SPDX-License-Identifier: Apache-2.0 */

#include "support_force_estimator.h"

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>

#include "chassis_config.h"
#include "gas_spring_model.h"

namespace {
// 本模块使用的固定算法参数，调整时与对应控制逻辑一起审查。
constexpr double kSupportWorldVerticalSpecificForceSign = 1;
constexpr double kSupportLeftJointTorqueSign = 1;
constexpr double kSupportRightJointTorqueSign = 1;
constexpr double kSupportDerivativeFilterCutoffHz = 18.0;
constexpr double kSupportForceFilterCutoffHz = 25.0;
constexpr double kSupportJacobianDamping = 1e-06;
constexpr double kSupportMaximumAbsForce = 300.0;
}  // namespace

namespace dust_control::modules {
namespace {
using namespace chassis_config;

double LowPassAlpha(double cutoff_hz, double dt) {
    constexpr double kPi = 3.14159265358979323846;
    return 1.0 - std::exp(-2.0 * kPi * cutoff_hz * dt);
}
}  // namespace

void SupportForceEstimator::Reset() {
    side_state_ = {};
    initialized_ = false;
}

ChassisSupportForceEstimate SupportForceEstimator::Update(
    const ChassisCycleInput &input) {
    using namespace chassis_config;
    ChassisSupportForceEstimate output = {};
    if (!input.feedback_valid || !input.imu_fresh) {
        Reset();
        return output;
    }

    const double dt = std::clamp(input.dt, 0.0005, 0.01);
    const double cr = std::cos(input.roll);
    const double sr = std::sin(input.roll);
    const double cp = std::cos(input.pitch);
    const double sp = std::sin(input.pitch);
    const double world_specific_force_g =
        kSupportWorldVerticalSpecificForceSign *
        (-sp * static_cast<double>(input.imu.accel_g[0]) +
         sr * cp * static_cast<double>(input.imu.accel_g[1]) +
         cr * cp * static_cast<double>(input.imu.accel_g[2]));
    output.world_vertical_specific_force_g = world_specific_force_g;
    output.body_vertical_accel_mps2 = (world_specific_force_g - 1.0) * kGravity;
    if (!std::isfinite(output.body_vertical_accel_mps2)) {
        Reset();
        return output;
    }

    const double derivative_alpha =
        LowPassAlpha(kSupportDerivativeFilterCutoffHz, dt);
    const double force_alpha = LowPassAlpha(kSupportForceFilterCutoffHz, dt);
    bool valid = true;
    for (Side side : {Side::kLeft, Side::kRight}) {
        const LegKinematics &leg = input.leg.Get(side);
        if (!std::isfinite(leg.length) || leg.length <= 1.0e-6) {
            valid = false;
            continue;
        }
        SideState &state = side_state_.Get(side);
        if (!initialized_) {
            state.previous_length_rate = leg.length_rate;
            state.previous_theta_rate = input.theta_rate.Get(side);
        }
        const double raw_length_accel =
            (leg.length_rate - state.previous_length_rate) / dt;
        const double raw_theta_accel =
            (input.theta_rate.Get(side) - state.previous_theta_rate) / dt;
        state.previous_length_rate = leg.length_rate;
        state.previous_theta_rate = input.theta_rate.Get(side);
        state.length_accel +=
            derivative_alpha * (raw_length_accel - state.length_accel);
        state.theta_accel +=
            derivative_alpha * (raw_theta_accel - state.theta_accel);

        Eigen::Matrix2d jacobian;
        jacobian << leg.jacobian[0][0], leg.jacobian[0][1], leg.jacobian[1][0],
            leg.jacobian[1][1];
        const JointPair<double> compensation =
            ConfiguredGasSpringCompensation(side, input, config_);
        const JointPair<double> &feedback =
            input.joint_torque_feedback.Get(side);
        const double torque_sign = side == Side::kLeft
                                       ? kSupportLeftJointTorqueSign
                                       : kSupportRightJointTorqueSign;
        // 机构净力矩 = 校准后的电机反馈 + 弹簧被动力矩。
        // 补偿力矩与被动力矩符号相反，列顺序为 [D, B]。
        const Eigen::Vector2d joint_torque(
            torque_sign * feedback.d - compensation.d,
            torque_sign * feedback.b - compensation.b);
        const Eigen::Matrix2d normal =
            jacobian * jacobian.transpose() +
            kSupportJacobianDamping * Eigen::Matrix2d::Identity();
        const Eigen::Vector2d endpoint_force =
            normal.ldlt().solve(jacobian * joint_torque);
        // 雅可比 J 和弹簧力矩使用原始电机/运动学坐标。
        // 先将末端力分解为径向和原始角向分量。
        // 校准后的机体角为 alpha = sign * (raw_angle - offset)，
        // 因此根据虚功原理，切向力必须应用相同的符号。
        const LegHardwareMap &hardware =
            config_.leg_hardware(side == Side::kRight);
        const double radial_x = leg.hx / leg.length;
        const double radial_z = leg.hz / leg.length;
        const double radial_force =
            radial_x * endpoint_force.x() + radial_z * endpoint_force.y();
        const double tangential_force =
            hardware.leg_coordinate_sign *
            (-radial_z * endpoint_force.x() + radial_x * endpoint_force.y());
        const double alpha = hardware.leg_coordinate_sign *
                             std::remainder(std::atan2(leg.hx, -leg.hz) -
                                                hardware.leg_angle_offset,
                                            kTwoPi);
        const double body_force_x =
            radial_force * std::sin(alpha) + tangential_force * std::cos(alpha);
        const double body_force_z = -radial_force * std::cos(alpha) +
                                    tangential_force * std::sin(alpha);
        // 末端驱动力表示机体对轮的作用力，其反作用力支撑机体。
        // 使用与上述 IMU 投影相同的机体到世界坐标竖直分量。
        const double leg_vertical_force =
            -(-sp * body_force_x + cr * cp * body_force_z);

        // 轮惯性保留矢状面近似 z_rel=-L*cos(theta)，
        // 此处不建模横滚及髋部侧向偏移引起的加速度。
        const double theta = input.theta.Get(side);
        const double theta_rate = input.theta_rate.Get(side);
        const double wheel_vertical_accel =
            output.body_vertical_accel_mps2 -
            state.length_accel * std::cos(theta) +
            2.0 * leg.length_rate * theta_rate * std::sin(theta) +
            leg.length * state.theta_accel * std::sin(theta) +
            leg.length * theta_rate * theta_rate * std::cos(theta);
        const double raw_force =
            leg_vertical_force +
            config_.support_wheel_mass * (kGravity + wheel_vertical_accel);
        if (!endpoint_force.allFinite() || !std::isfinite(raw_force)) {
            valid = false;
            continue;
        }
        const double bounded_force = std::clamp(
            raw_force, -kSupportMaximumAbsForce, kSupportMaximumAbsForce);
        if (!initialized_) {
            state.filtered_force = bounded_force;
        } else {
            state.filtered_force +=
                force_alpha * (bounded_force - state.filtered_force);
        }
        output.raw_force_n.Get(side) = raw_force;
        output.filtered_force_n.Get(side) = state.filtered_force;
        output.leg_vertical_force_n.Get(side) = leg_vertical_force;
        output.wheel_vertical_accel_mps2.Get(side) = wheel_vertical_accel;
    }

    initialized_ = valid;
    output.valid = valid;
    return output;
}

}  // namespace dust_control::modules
