/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis_controller.h"

#include <algorithm>

#include "chassis_config.h"

namespace dust_control::chassis {
namespace {
enum class ChassisJoint : uint8_t { kLeftB, kLeftD, kRightB, kRightD };
template <typename Enum>
constexpr size_t ToIndex(Enum value) {
    return static_cast<size_t>(value);
}
}  // namespace
using namespace chassis_config;
ChassisController::ChassisController(InputDeadlines deadlines,
                                     double wheel_torque_limit,
                                     ChassisConfig config)
    : recovery_controller_(config),
      climb_stairs_controller_(config),
      balance_controller_(wheel_torque_limit, config),
      input_reader_(deadlines, config),
      support_force_estimator_(config),
      flight_controller_(config),
      jump_controller_(config),
      state_machine_(config),
      output_policy_(config) {
    ValidateChassisConfig(config);
}

ChassisControlOutput ChassisController::Tick(
    const ControllerInputs& sources, uint64_t now_us, bool operator_permission,
    ChassisCycleInput* observed_input) {
    ChassisCycleInput input{};
    const double dt = last_us_ && now_us > last_us_
                          ? std::clamp((now_us - last_us_) * 1e-6, 0.0005, 0.01)
                          : kDefaultDt;
    last_us_ = now_us;
    input_reader_.Read(sources, input, now_us, dt,
                       output_policy_.arm_complete());
    input.requested_enable = input.requested_enable && operator_permission;
    const auto support = support_force_estimator_.Update(input);
    state_machine_.SetFlightDetected(flight_controller_.UpdateLiftoffDetection(
        input, support,
        state_machine_.state() == ChassisControlState::kBalance));
    UpdateControlState(input);
    ChassisControlOutput output{};
    // 动作的完成/失败在本周期反馈，下一周期 Update 才切换状态。
    ComputeControlOutput(input, output);
    if (output_policy_.Apply(state_machine_.state(), loop_ticks_, input,
                             output)) {
        balance_controller_.Reset();
    }
    ++loop_ticks_;
    if (observed_input) {
        *observed_input = input;
    }

    return output;
}

void ChassisController::UpdateControlState(const ChassisCycleInput& input) {
    const ChassisStateTransition transition = state_machine_.Update(input);
    if (transition.enable_changed && transition.enabled) {
        // 即使安全条件使使能后的首周期直接进入停止状态，
        // 仍保留使能边沿的初始化。
        balance_controller_.ResetForEnable();
    }

    if (!transition.changed) {
        return;
    }

    // 状态进入动作集中在此处；状态机只负责转换策略，
    // 不直接操作控制器或硬件。
    switch (transition.to) {
        case ChassisControlState::kDisabled:
        case ChassisControlState::kSafetyStop:
        case ChassisControlState::kActionFault:
        case ChassisControlState::kTiltFault:
        case ChassisControlState::kDmArming:
            balance_controller_.Reset();
            output_policy_.ResetArming();
            recovery_controller_.Reset();
            flight_controller_.Reset();
            jump_controller_.Reset();
            climb_stairs_controller_.Reset();
            break;
        case ChassisControlState::kWaitingFeedback:
            balance_controller_.Reset();
            break;
        case ChassisControlState::kRecovery:
            balance_controller_.Reset();
            recovery_controller_.Reset();
            break;
        case ChassisControlState::kBalance:
            if (transition.from == ChassisControlState::kRecovery ||
                transition.from == ChassisControlState::kClimbStairs ||
                transition.from == ChassisControlState::kFlight ||
                transition.from == ChassisControlState::kJump) {
                balance_controller_.SynchronizeReferences(input);
                recovery_controller_.Reset();
                climb_stairs_controller_.Reset();
                flight_controller_.Reset();
                jump_controller_.Reset();
            }
            break;
        case ChassisControlState::kFlight:
            flight_controller_.Reset();
            break;
        case ChassisControlState::kJump:
            balance_controller_.SynchronizeReferences(input);
            jump_controller_.Reset();
            break;
        case ChassisControlState::kClimbStairs:
            climb_stairs_controller_.Reset();
            break;
    }
}

void ChassisController::ComputeControlOutput(const ChassisCycleInput& input,
                                             ChassisControlOutput& output) {
    output = {};
    switch (state_machine_.state()) {
        case ChassisControlState::kDisabled:
        case ChassisControlState::kSafetyStop:
        case ChassisControlState::kDmArming:
        case ChassisControlState::kWaitingFeedback:
        case ChassisControlState::kActionFault:
        case ChassisControlState::kTiltFault:
            return;
        case ChassisControlState::kRecovery:
            UpdateRecovery(input, output);
            return;
        case ChassisControlState::kBalance:
            output.joint_control_active = true;
            balance_controller_.Update(input, output);
            return;
        case ChassisControlState::kFlight: {
            const auto flight = flight_controller_.Update(input);
            output = flight.control;
            output.joint_control_active = !flight.failed && !flight.finished;
            if (flight.failed) {
                state_machine_.MarkFlightFailed();
            } else if (flight.finished) {
                state_machine_.MarkFlightCompleted();
            }
            return;
        }
        case ChassisControlState::kJump: {
            const auto jump =
                jump_controller_.Update(input, balance_controller_);
            output = jump.control;
            output.joint_control_active = !jump.failed && !jump.finished;
            if (jump.failed) {
                state_machine_.MarkJumpFailed();
            } else if (jump.finished) {
                state_machine_.MarkJumpCompleted();
            }
            return;
        }
        case ChassisControlState::kClimbStairs:
            UpdateClimbStairs(input, output);
            return;
    }
}

void ChassisController::UpdateRecovery(const ChassisCycleInput& input,
                                       ChassisControlOutput& output) {
    RecoveryControllerInput recovery_input = {};
    recovery_input.dt = input.dt;
    recovery_input.pitch = input.pitch;
    recovery_input.roll = input.roll;
    recovery_input.vertical_accel_g = input.imu.vertical_accel_g;
    recovery_input.leg = {input.leg.left, input.leg.right};
    recovery_input.joint_position = {
        input.joint_position.left.b,
        input.joint_position.left.d,
        input.joint_position.right.b,
        input.joint_position.right.d,
    };
    recovery_input.joint_velocity = {
        input.joint_velocity.left.b,
        input.joint_velocity.left.d,
        input.joint_velocity.right.b,
        input.joint_velocity.right.d,
    };
    // 当前云台模块没有业务反馈；回正使用原有的无云台约束。
    recovery_input.gimbal.available = false;
    recovery_input.gimbal.safe_for_leg_motion = true;
    recovery_input.gimbal.aligned = true;
    const RecoveryControllerOutput recovery =
        recovery_controller_.Update(recovery_input);
    output.joint_control_active = recovery.joint_control_active;
    output.joint_torque.left.b =
        recovery.joint_torque[ToIndex(ChassisJoint::kLeftB)];
    output.joint_torque.left.d =
        recovery.joint_torque[ToIndex(ChassisJoint::kLeftD)];
    output.joint_torque.right.b =
        recovery.joint_torque[ToIndex(ChassisJoint::kRightB)];
    output.joint_torque.right.d =
        recovery.joint_torque[ToIndex(ChassisJoint::kRightD)];
    if (recovery.failed) {
        state_machine_.MarkRecoveryFailed();
    } else if (recovery.finished) {
        state_machine_.MarkRecoveryCompleted();
    }
}

void ChassisController::UpdateClimbStairs(const ChassisCycleInput& input,
                                          ChassisControlOutput& output) {
    ClimbStairsControllerInput climb_input = {};
    climb_input.dt = input.dt;
    climb_input.pitch = input.pitch;
    climb_input.leg = {input.leg.left, input.leg.right};
    climb_input.joint_position = {
        input.joint_position.left.b,
        input.joint_position.left.d,
        input.joint_position.right.b,
        input.joint_position.right.d,
    };
    climb_input.joint_velocity = {
        input.joint_velocity.left.b,
        input.joint_velocity.left.d,
        input.joint_velocity.right.b,
        input.joint_velocity.right.d,
    };
    const ClimbStairsControllerOutput climb =
        climb_stairs_controller_.Update(climb_input);
    output.joint_control_active = climb.joint_control_active;
    output.joint_torque.left.b =
        climb.joint_torque[ToIndex(ChassisJoint::kLeftB)];
    output.joint_torque.left.d =
        climb.joint_torque[ToIndex(ChassisJoint::kLeftD)];
    output.joint_torque.right.b =
        climb.joint_torque[ToIndex(ChassisJoint::kRightB)];
    output.joint_torque.right.d =
        climb.joint_torque[ToIndex(ChassisJoint::kRightD)];
    if (climb.failed) {
        state_machine_.MarkClimbFailed();
    } else if (climb.finished) {
        state_machine_.MarkClimbCompleted();
    }
}

}  // namespace dust_control::chassis
