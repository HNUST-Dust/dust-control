/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "chassis_output_policy.h"

#include "chassis_config.h"
#include "joint_output.h"

namespace {
// 本模块使用的固定算法参数，调整时与对应控制逻辑一起审查。
constexpr uint32_t kDmClearTicks = 50;
constexpr uint32_t kDmArmTicks = 300;
constexpr uint32_t kDmModePeriodTicks = 10;
static_assert(kDmClearTicks < kDmArmTicks);
}  // namespace

namespace dust_control::chassis {
namespace cfg = chassis_config;
namespace {
// 目标槽位与反馈顺序一致；编码器从 YAML 加载的配置取得 CAN 总线及 ID。
void SetJointTargets(dust_control::devices::MotorTargets& targets,
                     dust_control::devices::MotorMode mode,
                     const SidePair<JointPair<double>>& torque = {}) {
    targets[2] = {mode, torque.left.b};
    targets[3] = {mode, torque.left.d};
    targets[4] = {mode, torque.right.b};
    targets[5] = {mode, torque.right.d};
}

void SetWheelTargets(dust_control::devices::MotorTargets& targets,
                     const SidePair<double>& torque = {}) {
    targets[0] = {dust_control::devices::MotorMode::kTorque, torque.left};
    targets[1] = {dust_control::devices::MotorMode::kTorque, torque.right};
}
}  // namespace

bool ChassisOutputPolicy::arm_complete() const {
    return dm_arm_ticks_ >= kDmArmTicks;
}

bool ChassisOutputPolicy::Apply(ChassisControlState state, uint32_t loop_ticks,
                                const ChassisCycleInput& input,
                                ChassisControlOutput& output) {
    PrepareJointOutput(state, input, output, config_);
    auto& targets = output.motor_targets;
    targets = {};
    switch (state) {
        case ChassisControlState::kDisabled:
        case ChassisControlState::kSafetyStop:
        case ChassisControlState::kActionFault:
        case ChassisControlState::kTiltFault:
            if ((loop_ticks % 100U) == 0U) {
                SetJointTargets(targets,
                                dust_control::devices::MotorMode::kDisable);
            }
            if ((loop_ticks % 10U) == 0U) {
                SetWheelTargets(targets);
            }
            dm_arm_ticks_ = 0U;
            return true;

        case ChassisControlState::kDmArming:
            if ((loop_ticks % kDmModePeriodTicks) == 0U) {
                SetJointTargets(
                    targets, dm_arm_ticks_ < kDmClearTicks
                                 ? dust_control::devices::MotorMode::kClearError
                                 : dust_control::devices::MotorMode::kEnable);
            }
            SetWheelTargets(targets);
            if (dm_arm_ticks_ < kDmArmTicks) {
                ++dm_arm_ticks_;
            }
            return true;

        case ChassisControlState::kWaitingFeedback:
            SetJointTargets(targets, dust_control::devices::MotorMode::kTorque);
            SetWheelTargets(targets);
            return true;

        case ChassisControlState::kRecovery:
        case ChassisControlState::kClimbStairs:
            SetJointTargets(targets, dust_control::devices::MotorMode::kTorque,
                            output.joint_torque);
            SetWheelTargets(targets);
            return false;

        case ChassisControlState::kBalance:
        case ChassisControlState::kFlight:
        case ChassisControlState::kJump:
            SetJointTargets(targets, dust_control::devices::MotorMode::kTorque,
                            output.joint_torque);
            SetWheelTargets(targets, output.physical_wheel_torque);
            return false;
    }

    return true;
}

}  // namespace dust_control::chassis
