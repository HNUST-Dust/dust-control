/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "chassis_config.h"
#include "chassis_types.h"

namespace dust_control::chassis {

enum class ChassisTransitionReason : uint8_t {
    kNone = 0U,
    kRemoteDisabled,
    kRemoteTimeout,
    kRemoteEnabled,
    kTiltExceeded,
    kImuLost,
    kImuRecovered,
    kDmNotReady,
    kArmingCompleted,
    kFeedbackLost,
    kFeedbackRecovered,
    kRecoveryStarted,
    kRecoveryCompleted,
    kRecoveryFailed,
    kFlightDetected,
    kFlightLanded,
    kFlightFailed,
    kJumpRequested,
    kJumpCompleted,
    kJumpFailed,
    kClimbRequested,
    kClimbCompleted,
    kClimbFailed,
};

struct ChassisStateTransition {
    ChassisControlState from = ChassisControlState::kDisabled;
    ChassisControlState to = ChassisControlState::kDisabled;
    ChassisTransitionReason reason = ChassisTransitionReason::kNone;
    bool changed = false;
    bool enable_changed = false;
    bool enabled = false;
};

/** 保存故障锁存和动作进度，决定底盘运行状态转换。 */
class ChassisStateMachine {
   public:
    explicit ChassisStateMachine(ChassisConfig config = {}) : config_(config) {}
    ChassisStateTransition Update(const ChassisCycleInput &input);
    // 动作在计算输出后写入反馈，下一次 Update 读取；标志不在读取时消费。
    void MarkRecoveryCompleted() {
        recovery_completed_ = true;
    }

    void MarkRecoveryFailed() {
        recovery_failed_ = true;
    }

    void MarkClimbCompleted() {
        climb_completed_ = true;
    }

    void MarkClimbFailed() {
        climb_failed_ = true;
    }

    void SetFlightDetected(bool detected) {
        flight_detected_ = detected;
    }

    void MarkFlightCompleted() {
        flight_completed_ = true;
    }

    void MarkFlightFailed() {
        flight_failed_ = true;
    }

    void MarkJumpCompleted() {
        jump_completed_ = true;
    }

    void MarkJumpFailed() {
        jump_failed_ = true;
    }

    void SetArmResetReason(DmArmResetReason reason) {
        arm_reset_reason_ = reason;
    }

    ChassisControlState state() const {
        return state_;
    }

    bool tilt_fault_latched() const {
        return tilt_fault_latched_;
    }

    DmArmResetReason arm_reset_reason() const {
        return arm_reset_reason_;
    }

   private:
    void UpdateSafetyLatches(const ChassisCycleInput &input);
    void UpdateRequests(const ChassisCycleInput &input,
                        ChassisStateTransition &transition);
    ChassisControlState DetermineNextState(
        const ChassisCycleInput &input, ChassisTransitionReason &reason) const;
    ChassisControlState DetermineActionState(
        ChassisTransitionReason &reason) const;
    void ApplyTransition(const ChassisStateTransition &transition);

    const ChassisConfig config_;
    ChassisControlState state_ = ChassisControlState::kDisabled;
    DmArmResetReason arm_reset_reason_ = DmArmResetReason::kStartup;
    bool last_requested_enable_ = false;

    // 只有新鲜遥控的明确禁用才统一清锁存；动作失败在转换判断后锁存。
    bool tilt_fault_latched_ = false;
    bool action_fault_latched_ = false;

    // 使能边沿要求回正；回正成功交接平衡后清 required/completed。
    bool recovery_required_ = false;
    bool recovery_completed_ = false;
    bool recovery_failed_ = false;

    // 离地检测由总控每周期提供；动作反馈在重新进入或明确禁用时清除。
    bool flight_detected_ = false;
    bool flight_completed_ = false;
    bool flight_failed_ = false;

    // 请求边沿每周期更新，反馈在重新进入或明确禁用时清除。
    bool last_jump_request_ = false;
    bool jump_request_edge_ = false;
    bool jump_completed_ = false;
    bool jump_failed_ = false;
    double jump_cooldown_remaining_s_ = 0.0;

    bool last_climb_request_ = false;
    bool climb_request_edge_ = false;
    bool climb_completed_ = false;
    bool climb_failed_ = false;
};

}  // namespace dust_control::chassis
