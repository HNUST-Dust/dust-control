/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <cstdint>

namespace dust::devices {
// 设备驱动产生的物理反馈；时间戳使用主机单调时钟。
struct MotorFeedback {
    uint64_t precise_timestamp_us = 0;
    bool valid = false;
    double position_rad = 0.0;
    double velocity_rad_s = 0.0;
    double torque_nm = 0.0;
    double speed_rpm = 0.0;
    bool enabled = false;
};
}  // namespace dust::devices
