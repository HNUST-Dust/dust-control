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
