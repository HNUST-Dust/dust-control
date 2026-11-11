/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <optional>

#include "body_motion_estimator.h"
#include "chassis_config.h"
#include "chassis_types.h"
#include "lqr14_schedule.h"

namespace dust_control::chassis {

/** 维护平衡参考和滤波状态；通过冻结输入计算输出，不执行设备 I/O。 */
class BalanceController {
   public:
    explicit BalanceController(
        double wheel_torque_limit = chassis_config::kWheelTorqueLimit,
        ChassisConfig config = {});
    // 清空运动参考、估计、yaw 和腿长积分历史；保留腿长和姿态参考。
    // 停止、使能握手或等待反馈期间可重复调用；不等同于重新使能初始化。
    void Reset();
    // 使能边沿：恢复配置腿长、平衡腿角及零 pitch/roll，再清空历史。
    void ResetForEnable();
    // 动作交接：从当前双腿平均长度、common_theta、pitch/roll 接续，再清空历史。
    // yaw 参考不在这里采样，而是在下一次 Update 中初始化。
    void SynchronizeReferences(const ChassisCycleInput &input);
    void Update(const ChassisCycleInput &input, ChassisControlOutput &output,
                const ExternalSupportForceCommand *external_support = nullptr,
                bool allow_lqr14 = true);

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
    void AllocateBalanceTorques(double left_wheel, double right_wheel,
                                double left_leg, double right_leg,
                                ChassisControlOutput &output) const;
    bool ComputeLqr14Output(const ChassisCycleInput &input,
                            const MotionTracking &tracking,
                            ChassisControlOutput &candidate,
                            const ExternalSupportForceCommand *external_support = nullptr,
                            bool allow_extensions = true);
    // 每侧独立维护腿长积分，再将支撑力和姿态力矩映射为关节力矩。
    void ComputeLegOutput(const ChassisCycleInput &input, Side side,
                          double target_leg_length_rate,
                          const ExternalSupportForceCommand *external_support,
                          ChassisControlOutput &output);

    // 构造后固定，不参与复位。
    const ChassisConfig config_;
    const double wheel_torque_limit_;

    // 腿长与姿态参考：Reset 保留，使能/动作交接入口负责重新设置。
    double target_leg_length_ = 0.0;
    double balance_theta_reference_ = 0.0;
    double balance_pitch_reference_ = 0.0;
    double balance_roll_reference_ = 0.0;

    // 运动估计与平移参考：Reset 同时清空估计位移及目标位移。
    BodyMotionEstimator body_motion_estimator_;
    double target_body_speed_ = 0.0;
    double target_body_position_ = 0.0;

    // yaw 参考：Reset 清空，下一次 Update 用新采样初始化目标角。
    double target_yaw_rate_ = 0.0;
    double target_yaw_angle_ = 0.0;
    bool yaw_reference_initialized_ = false;

    // yaw 反馈滤波：与目标参考独立初始化，Reset 后首个采样直接赋值。
    double filtered_yaw_rate_ = 0.0;
    bool yaw_rate_filter_initialized_ = false;

    std::optional<Lqr14GainSchedule> lqr14_schedule_;
    double lqr14_support_elapsed_s_ = 0.0;
    double lqr14_blend_ = 0.0;
    double lqr14_filtered_height_rate_ = 0.0;
    bool lqr14_height_initialized_ = false;
    double lqr14_previous_height_reference_ = 0.0;

    // 每侧腿长积分：Reset 清空；外部支撑力启用时在单腿计算中清空。
    SidePair<SideState> side_state_;
};

}  // namespace dust_control::chassis
