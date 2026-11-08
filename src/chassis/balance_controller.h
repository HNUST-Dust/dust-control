/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "body_motion_estimator.h"
#include "chassis_config.h"
#include "chassis_types.h"

namespace dust_control::chassis {

/** 维护平衡参考和滤波状态；通过冻结输入计算输出，不执行设备 I/O。 */
class BalanceController {
   public:
    explicit BalanceController(
        double wheel_torque_limit = chassis_config::kWheelTorqueLimit,
        ChassisConfig config = {});
    void Reset();
    void ResetForEnable();
    void SynchronizeReferences(const ChassisCycleInput &input);
    void Update(const ChassisCycleInput &input, ChassisControlOutput &output,
                const ExternalSupportForceCommand *external_support = nullptr);

    void set_target_leg_length(double value) {
        target_leg_length_ = value;
    }

    double target_leg_length() const {
        return target_leg_length_;
    }

    double target_yaw_rate() const {
        return target_yaw_rate_;
    }

    double target_yaw_angle() const {
        return target_yaw_angle_;
    }

    double filtered_yaw_rate() const {
        return filtered_yaw_rate_;
    }

   private:
    const ChassisConfig config_;
    struct SideState {
        double leg_length_integral_force = 0.0;
    };

    // 本周期运动跟踪误差和腿长参考速度，不保存为跨周期状态。
    struct MotionTracking {
        double position_error;
        double speed_error;
        double yaw_error;
        double yaw_rate_error;
        double target_leg_length_rate;
    };

    // 参考更新维护渐变姿态；运动估计维护滤波和位移。
    void UpdateAttitudeReferences(
        const ChassisCycleInput &input,
        const ExternalSupportForceCommand *external_support,
        ChassisControlOutput &output);
    const BodyMotionState &EstimateBodyMotion(const ChassisCycleInput &input,
                                              ChassisControlOutput &output);
    // 更新遥控参考及 yaw 滤波，形成 LQR 所需的运动跟踪误差。
    MotionTracking UpdateMotionReferences(const ChassisCycleInput &input,
                                          const BodyMotionState &motion);
    void ComputeBalanceTorques(const ChassisCycleInput &input,
                               const MotionTracking &tracking,
                               ChassisControlOutput &output) const;
    // 每侧独立维护腿长积分，再将支撑力和姿态力矩映射为关节力矩。
    void ComputeLegOutput(const ChassisCycleInput &input, Side side,
                          double target_leg_length_rate,
                          const ExternalSupportForceCommand *external_support,
                          ChassisControlOutput &output);

    const double wheel_torque_limit_;
    BodyMotionEstimator body_motion_estimator_;
    double target_leg_length_ = 0.0;
    double balance_theta_reference_ = 0.0;
    double balance_pitch_reference_ = 0.0;
    double balance_roll_reference_ = 0.0;
    double target_body_speed_ = 0.0;
    double target_body_position_ = 0.0;
    double target_yaw_rate_ = 0.0;
    double target_yaw_angle_ = 0.0;
    bool yaw_reference_initialized_ = false;
    double filtered_yaw_rate_ = 0.0;
    bool yaw_rate_filter_initialized_ = false;
    SidePair<SideState> side_state_;
};

}  // namespace dust_control::chassis
