/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <algorithm>
#include <cmath>

#include "chassis/controller_inputs.h"
#include "chassis_config.h"
#include "motion_command.hpp"

namespace dust::chassis {
inline double ToRemoteAxis(double value, double maximum) {
    using namespace modules::chassis_config;
    const double fraction = std::clamp(value / maximum, -1.0, 1.0);
    return fraction == 0
               ? 0
               : std::copysign(kRemoteDeadband +
                                   std::abs(fraction) * (1 - kRemoteDeadband),
                               fraction);
}
// 保留物理遥控的 sequence、使能和失能姿态，外部指令不能刷新遥控新鲜度或使能。
inline void ApplyMotionCommand(modules::ControllerInputs& input,
                               MotionCommand& command,
                               const MotionCommandOptions& options,
                               MotionCommandQueue& queue, uint64_t now_ns) {
    queue.TakeLatest(command);
    if (!options.external_commands) {
        return;
    }
    using namespace modules::chassis_config;
    auto& remote = input.remote;
    const auto active = [](float axis) {
        return std::abs(axis) > kRemoteDeadband;
    };
    if (!remote.robot_enable || remote.jump || remote.climb_stairs ||
        remote.fast_spin || active(remote.chassis_x) ||
        active(remote.chassis_rotate) || active(remote.leg_length_delta) ||
        active(remote.pitch_angle) || active(remote.yaw_angle)) {
        // 手动干预后必须收到新外部指令，不能恢复干预前缓存的目标。
        command = {};
        return;
    }
    const bool fresh = command.received_ns && now_ns >= command.received_ns &&
                       now_ns - command.received_ns < options.timeout_ns;
    remote.chassis_x =
        fresh ? ToRemoteAxis(command.forward_m_s, kMaxBodySpeed) : 0;
    // 现有遥控轴顺时针为正，ROS yaw 采用逆时针为正。
    remote.chassis_rotate =
        fresh ? ToRemoteAxis(-command.yaw_rad_s, kMaxYawRate) : 0;
    remote.leg_length_delta =
        fresh ? ToRemoteAxis(command.leg_rate_m_s, kLegLengthCommandRate) : 0;
}
}  // namespace dust::chassis
