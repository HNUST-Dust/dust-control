/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstdint>
#include <string>

#include "motor_map.h"

namespace dust::devices {
struct MotorConfig {
    std::array<platform::motor_map::MotorAddress, 6> addresses =
        platform::motor_map::kDefaultMotors;
    std::array<uint64_t, 6> freshness_us{3000, 3000, 3000, 3000, 3000, 3000};
};
// 启动时返回完整校验的配置值，不修改任何已存在的设备实例。
MotorConfig LoadMotorConfig(const std::string& file);
}  // namespace dust::devices
