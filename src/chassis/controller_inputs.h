/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <hardware/motors/motor_protocol.hpp>
#include <hardware/remote/remote_input_state.hpp>

#include "hardware/imu/hi91_protocol.h"

namespace dust_control::modules {
// 控制线程组合后冻结的输入；设备独立采样，时间戳均使用主机单调时钟。
struct ControllerInputs {
    std::array<dust_control::devices::MotorFeedback, 6> motors{};
    dust_control::input::RemoteInputState remote{};
    dust_control::devices::ImuSample imu{};
    bool remote_disable_pose = false;
};
}  // namespace dust_control::modules
