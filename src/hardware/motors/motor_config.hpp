/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstdint>
#include <string>

#include "dji_model.hpp"
#include "dm_model.hpp"
#include "motor_map.h"

namespace dust::devices {
struct MotorConfig {
    std::array<platform::motor_map::MotorAddress, 6> addresses =
        platform::motor_map::kDefaultMotors;
    std::array<uint64_t, 6> freshness_us{3000, 3000, 3000, 3000, 3000, 3000};
    std::array<double, 2> wheel_command_sign{-1.0, 1.0};
    double dji_current_per_nm = platform::motor_model::kDefaultDjiCurrentPerNm;
    static constexpr int16_t dji_protocol_current_limit =
        platform::motor_model::kDjiProtocolCurrentLimit;
    static constexpr protocols::DmMitRange dm_range =
        platform::motor_model::kJointMitRange;
    double wheel_torque_limit_nm() const {
        return static_cast<double>(dji_protocol_current_limit) /
               dji_current_per_nm;
    }
};
void ValidateMotorConfig(const MotorConfig& config);
// 启动时返回完整校验的配置值，不修改任何已存在的设备实例。
MotorConfig LoadMotorConfig(const std::string& file);
}  // namespace dust::devices
