/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <dust_control/msg/chassis_telemetry.hpp>
#include <rclcpp/time.hpp>

#include "chassis/chassis_observation.hpp"

namespace dust::ros {
// 仅由非实时发布回调调用；复制观测值，按样本年龄标记失效数据。
dust_control::msg::ChassisTelemetry MakeChassisTelemetry(
    const chassis::ChassisObservation& sample, const rclcpp::Time& now,
    uint64_t monotonic_ns);
}  // namespace dust::ros
