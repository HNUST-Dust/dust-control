/* SPDX-License-Identifier: Apache-2.0 */

#include "chassis/control/chassis_state_machine.h"

#include <algorithm>
#include <cmath>

#include "chassis_config.h"

namespace {
// 本模块使用的固定算法参数，调整时与对应控制逻辑一起审查。
constexpr uint32_t kJumpCooldownMs = 3000;
}  // namespace

namespace dust_control::chassis {

ChassisStateTransition ChassisStateMachine::Update(
    const ChassisCycleInput &input) {
    ChassisStateTransition transition;
    UpdateSafetyLatches(input);
    UpdateRequests(input, transition);

    transition.from = state_;
    transition.to = DetermineNextState(input, transition.reason);
    transition.changed = transition.to != transition.from;
    state_ = transition.to;
    ApplyTransition(transition);

    // 转换判断后才锁存失败；不要提前到本周期的公共保护条件中。
    if (climb_failed_ || recovery_failed_ || flight_failed_ || jump_failed_) {
        action_fault_latched_ = true;
    }

    if (state_ == ChassisControlState::kSafetyStop) {
        arm_reset_reason_ = DmArmResetReason::kImuFreshness;
    } else if (state_ == ChassisControlState::kTiltFault) {
        arm_reset_reason_ = DmArmResetReason::kTiltFault;
    }

    return transition;
}

void ChassisStateMachine::UpdateSafetyLatches(const ChassisCycleInput &input) {
    // 新鲜遥控的明确禁用统一清除锁存和动作反馈。
    // 遥控超时或外部许可撤回不等同于明确禁用，不能据此清除故障。
    if (input.remote_fresh && !input.remote.robot_enable) {
        tilt_fault_latched_ = false;
        action_fault_latched_ = false;
        climb_completed_ = false;
        climb_failed_ = false;
        flight_detected_ = false;
        flight_completed_ = false;
        flight_failed_ = false;
        jump_completed_ = false;
        jump_failed_ = false;
        recovery_required_ = false;
        recovery_completed_ = false;
        recovery_failed_ = false;
    }

    if (input.requested_enable && input.imu_fresh &&
        state_ != ChassisControlState::kRecovery && !recovery_required_) {
        const double pitch_deg = static_cast<double>(input.imu.pitch_deg);
        if (!tilt_fault_latched_ &&
            std::abs(pitch_deg) >= config_.tilt_cutoff_deg) {
            tilt_fault_latched_ = true;
        }
    }
}

void ChassisStateMachine::UpdateRequests(const ChassisCycleInput &input,
                                         ChassisStateTransition &transition) {
    // 每周期消费请求边沿；禁用、冷却或被更高优先级规则挡住时都不排队。
    climb_request_edge_ = input.remote.climb_stairs && !last_climb_request_;
    last_climb_request_ = input.remote.climb_stairs;
    jump_request_edge_ = input.remote.jump && !last_jump_request_;
    last_jump_request_ = input.remote.jump;
    jump_cooldown_remaining_s_ =
        std::max(0.0, jump_cooldown_remaining_s_ - input.dt);

    if (input.requested_enable != last_requested_enable_) {
        transition.enable_changed = true;
        transition.enabled = input.requested_enable;
        if (input.requested_enable) {
            recovery_required_ = true;
            recovery_completed_ = false;
            recovery_failed_ = false;
            arm_reset_reason_ = DmArmResetReason::kRemoteEnableEdge;
        }
        last_requested_enable_ = input.requested_enable;
    }
}

ChassisControlState ChassisStateMachine::DetermineNextState(
    const ChassisCycleInput &input, ChassisTransitionReason &reason) const {
    // 按代码顺序短路：禁用 > 已锁存动作故障 > IMU 丢失 > 倾倒锁存。
    // 公共保护通过后，才检查握手/反馈和各状态的正常转换。
    if (!input.requested_enable) {
        reason = input.remote_fresh ? ChassisTransitionReason::kRemoteDisabled
                                    : ChassisTransitionReason::kRemoteTimeout;
        return ChassisControlState::kDisabled;
    }

    if (action_fault_latched_) {
        // 多个失败标志并存时，诊断原因也有固定优先级。
        if (recovery_failed_) {
            reason = ChassisTransitionReason::kRecoveryFailed;
        } else if (flight_failed_) {
            reason = ChassisTransitionReason::kFlightFailed;
        } else if (jump_failed_) {
            reason = ChassisTransitionReason::kJumpFailed;
        } else {
            reason = ChassisTransitionReason::kClimbFailed;
        }
        return ChassisControlState::kActionFault;
    }

    if (!input.imu_fresh) {
        reason = ChassisTransitionReason::kImuLost;
        return ChassisControlState::kSafetyStop;
    }

    if (tilt_fault_latched_ && state_ != ChassisControlState::kRecovery &&
        !recovery_required_) {
        reason = ChassisTransitionReason::kTiltExceeded;
        return ChassisControlState::kTiltFault;
    }

    switch (state_) {
        case ChassisControlState::kDisabled:
            reason = ChassisTransitionReason::kRemoteEnabled;
            return ChassisControlState::kDmArming;
        case ChassisControlState::kSafetyStop:
            reason = ChassisTransitionReason::kImuRecovered;
            return ChassisControlState::kDmArming;
        case ChassisControlState::kDmArming:
            if (!input.arm_complete || !input.dm_ready) {
                return state_;
            }
            if (!input.feedback_valid) {
                reason = ChassisTransitionReason::kArmingCompleted;
                return ChassisControlState::kWaitingFeedback;
            }
            reason = ChassisTransitionReason::kArmingCompleted;
            if (recovery_required_) {
                reason = ChassisTransitionReason::kRecoveryStarted;
                return ChassisControlState::kRecovery;
            }
            return ChassisControlState::kBalance;
        case ChassisControlState::kWaitingFeedback:
            if (!input.arm_complete || !input.dm_ready) {
                reason = ChassisTransitionReason::kDmNotReady;
                return ChassisControlState::kDmArming;
            }
            if (!input.feedback_valid) {
                return state_;
            }
            reason = ChassisTransitionReason::kFeedbackRecovered;
            if (recovery_required_) {
                reason = ChassisTransitionReason::kRecoveryStarted;
                return ChassisControlState::kRecovery;
            }
            return ChassisControlState::kBalance;
        case ChassisControlState::kRecovery:
        case ChassisControlState::kBalance:
        case ChassisControlState::kFlight:
        case ChassisControlState::kJump:
        case ChassisControlState::kClimbStairs:
            if (!input.arm_complete || !input.dm_ready) {
                reason = ChassisTransitionReason::kDmNotReady;
                return ChassisControlState::kDmArming;
            }
            if (!input.feedback_valid) {
                reason = ChassisTransitionReason::kFeedbackLost;
                return ChassisControlState::kWaitingFeedback;
            }
            return DetermineActionState(reason);
        case ChassisControlState::kActionFault:
        case ChassisControlState::kTiltFault:
            return state_;
    }

    // 状态损坏或新增状态尚未处理时，默认进入安全停止。
    return ChassisControlState::kSafetyStop;
}

ChassisControlState ChassisStateMachine::DetermineActionState(
    ChassisTransitionReason &reason) const {
    switch (state_) {
        case ChassisControlState::kRecovery:
            if (recovery_failed_) {
                reason = ChassisTransitionReason::kRecoveryFailed;
                return ChassisControlState::kActionFault;
            }
            if (recovery_completed_) {
                reason = ChassisTransitionReason::kRecoveryCompleted;
                return ChassisControlState::kBalance;
            }
            break;
        case ChassisControlState::kBalance:
            // 同周期请求竞争：离地 > 可执行跳跃 > 爬楼。
            if (flight_detected_) {
                reason = ChassisTransitionReason::kFlightDetected;
                return ChassisControlState::kFlight;
            }
            if (jump_request_edge_ && jump_cooldown_remaining_s_ <= 0.0) {
                reason = ChassisTransitionReason::kJumpRequested;
                return ChassisControlState::kJump;
            }
            if (climb_request_edge_) {
                reason = ChassisTransitionReason::kClimbRequested;
                return ChassisControlState::kClimbStairs;
            }
            break;
        case ChassisControlState::kFlight:
            if (flight_failed_) {
                reason = ChassisTransitionReason::kFlightFailed;
                return ChassisControlState::kActionFault;
            }
            if (flight_completed_) {
                reason = ChassisTransitionReason::kFlightLanded;
                return ChassisControlState::kBalance;
            }
            break;
        case ChassisControlState::kJump:
            if (jump_failed_) {
                reason = ChassisTransitionReason::kJumpFailed;
                return ChassisControlState::kActionFault;
            }
            if (jump_completed_) {
                reason = ChassisTransitionReason::kJumpCompleted;
                return ChassisControlState::kBalance;
            }
            break;
        case ChassisControlState::kClimbStairs:
            if (climb_failed_) {
                reason = ChassisTransitionReason::kClimbFailed;
                return ChassisControlState::kActionFault;
            }
            if (climb_completed_) {
                reason = ChassisTransitionReason::kClimbCompleted;
                return ChassisControlState::kBalance;
            }
            break;
        default:
            break;  // 调用方只在握手及反馈有效的运行态进入此处。
    }
    return state_;
}

void ChassisStateMachine::ApplyTransition(
    const ChassisStateTransition &transition) {
    if (!transition.changed) {
        return;
    }
    if (transition.to == ChassisControlState::kClimbStairs) {
        climb_completed_ = false;
        climb_failed_ = false;
    }

    if (transition.to == ChassisControlState::kFlight) {
        flight_completed_ = false;
        flight_failed_ = false;
    }

    if (transition.to == ChassisControlState::kJump) {
        jump_completed_ = false;
        jump_failed_ = false;
    }

    if (transition.from == ChassisControlState::kJump) {
        // 无论完成、失败还是中断，只要离开跳跃就开始冷却；禁用不清冷却。
        jump_cooldown_remaining_s_ =
            static_cast<double>(kJumpCooldownMs) * 0.001;
    }

    if (transition.from == ChassisControlState::kRecovery &&
        transition.to == ChassisControlState::kBalance) {
        recovery_required_ = false;
        recovery_completed_ = false;
        tilt_fault_latched_ = false;
    }
}

}  // namespace dust_control::chassis
