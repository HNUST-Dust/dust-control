/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <hardware/motors/motor_state.hpp>
#include <hardware/remote/remote_input_state.hpp>

#include "hardware/imu/imu_sample.hpp"

namespace modules {
// 控制线程组合后冻结的输入；设备独立采样，时间戳均使用主机单调时钟。
struct ControllerInputs {
    std::array<dust::devices::MotorFeedback, 6> motors{};
    msg::RemoteInputState remote{};
    dust::devices::ImuSample imu{};
    bool remote_disable_pose = false;
};
}  // namespace modules
