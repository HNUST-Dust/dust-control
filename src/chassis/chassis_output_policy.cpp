/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "chassis_output_policy.h"

#include <algorithm>
#include <cmath>

#include "chassis_config.h"
#include "gas_spring.h"

namespace {
// 本模块使用的固定算法参数，调整时与对应控制逻辑一起审查。
constexpr uint32_t kDmClearTicks = 50;
constexpr uint32_t kDmArmTicks = 300;
constexpr uint32_t kDmModePeriodTicks = 10;
static_assert(kDmClearTicks < kDmArmTicks);
}  // namespace

namespace dust_control::chassis {
namespace {
// 仅由 ChassisOutputPolicy 在选择控制器后、提交设备目标前调用一次。
// 输入和输出均为电机坐标 N·m；先加弹簧补偿，再执行关节最终限幅。
void PrepareJointOutput(ChassisControlState state,
                        const ChassisCycleInput& input,
                        ChassisControlOutput& output,
                        const ChassisConfig& config) {
    using namespace chassis_config;
    bool active_state = false;
    switch (state) {
        case ChassisControlState::kBalance:
        case ChassisControlState::kRecovery:
        case ChassisControlState::kFlight:
        case ChassisControlState::kJump:
        case ChassisControlState::kClimbStairs:
            active_state = true;
            break;
        default:
            break;
    }

    if (!active_state || !output.joint_control_active ||
        !input.feedback_valid || !input.imu_fresh || !input.dm_ready ||
        !input.arm_complete || !input.requested_enable) {
        output.joint_torque = {};
        return;
    }

    const bool position_control = state == ChassisControlState::kRecovery ||
                                  state == ChassisControlState::kClimbStairs;
    const double limit =
        position_control ? std::min(config.joint_torque_limit, kJointTorqueMax)
                         : config.joint_torque_limit;
    for (Side side : {Side::kLeft, Side::kRight}) {
        const JointPair<double> compensation =
            state == ChassisControlState::kJump &&
                    output.bypass_gas_spring_compensation
                ? JointPair<double>{}
                : ConfiguredGasSpringCompensation(side, input, config);
        JointPair<double>& command = output.joint_torque.Get(side);
        for (Joint joint : {Joint::kB, Joint::kD}) {
            double& torque = command.Get(joint);
            // 控制器输出无效时，不能退化为只有弹簧补偿的命令。
            const double sum = torque + compensation.Get(joint);
            torque = std::isfinite(torque) && std::isfinite(sum)
                         ? std::clamp(sum, -limit, limit)
                         : 0.0;
        }
    }
}

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
    auto& idle_targets = output.idle_motor_targets;
    SetWheelTargets(idle_targets);
    SetJointTargets(idle_targets, dust_control::devices::MotorMode::kTorque);
    switch (state) {
        case ChassisControlState::kDisabled:
        case ChassisControlState::kSafetyStop:
        case ChassisControlState::kActionFault:
        case ChassisControlState::kTiltFault:
            SetJointTargets(idle_targets,
                            dust_control::devices::MotorMode::kDisable);
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
