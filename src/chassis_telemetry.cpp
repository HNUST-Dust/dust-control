/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis_telemetry.hpp"

#include <cmath>
#include <limits>

namespace dust::ros {
// 仅由非实时发布回调调用；按样本与设备新鲜度转换观测。
dust_control::msg::ChassisTelemetry MakeChassisTelemetry(
    const chassis::ChassisObservation& sample, const rclcpp::Time& now,
    uint64_t monotonic_ns) {
    dust_control::msg::ChassisTelemetry message;
    const auto& input = sample.input;
    const auto age = monotonic_ns >= sample.monotonic_ns
                         ? monotonic_ns - sample.monotonic_ns
                         : UINT64_MAX;
    const bool connected = age <= 200000000;
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    const auto number = [invalid](double value, bool valid) {
        return valid && std::isfinite(value) ? value : invalid;
    };
    message.header.stamp = now;
    if (age <= uint64_t(now.nanoseconds())) {
        message.header.stamp =
            now - rclcpp::Duration::from_nanoseconds(int64_t(age));
    }
    message.header.frame_id = "base_link";
    message.sequence = sample.sequence;
    message.monotonic_ns = sample.monotonic_ns;
    message.simulated = sample.simulated;
    message.source_connected = connected;
    message.imu_fresh = connected && input.imu_fresh;
    message.remote_fresh = connected && input.remote_fresh;
    message.feedback_valid = connected && input.feedback_valid;
    message.motor_stale_mask = connected ? input.motor_stale_mask : 0x3f;
    message.state = uint8_t(sample.state);
    message.missed_releases = sample.missed_releases;
    message.queue_drops = sample.queue_drops;
    message.transport_age_ms = age * 1e-6;
    message.control_duration_us = sample.control_duration_ns * 1e-3;
    message.imu_age_us = number(double(input.imu_age_us),
                                connected && input.imu_age_us != UINT64_MAX);
    message.pitch_deg = number(input.imu.pitch_deg, message.imu_fresh);
    message.roll_deg = number(input.imu.roll_deg, message.imu_fresh);
    message.yaw_deg = number(input.imu.yaw_deg, message.imu_fresh);
    message.left_leg_fresh = connected && !(input.motor_stale_mask & 0x0c) &&
                             !(input.feedback_invalid_mask & 4);
    message.right_leg_fresh = connected && !(input.motor_stale_mask & 0x30) &&
                              !(input.feedback_invalid_mask & 8);
    message.left_length_m =
        number(input.leg.left.length, message.left_leg_fresh);
    message.right_length_m =
        number(input.leg.right.length, message.right_leg_fresh);
    message.left_height_m = number(-input.leg.left.hz, message.left_leg_fresh);
    message.right_height_m =
        number(-input.leg.right.hz, message.right_leg_fresh);
    message.left_angle_rad =
        number(input.leg.left.angle, message.left_leg_fresh);
    message.right_angle_rad =
        number(input.leg.right.angle, message.right_leg_fresh);
    message.left_length_rate_m_s =
        number(input.leg.left.length_rate, message.left_leg_fresh);
    message.right_length_rate_m_s =
        number(input.leg.right.length_rate, message.right_leg_fresh);
    return message;
}
}  // namespace dust::ros
