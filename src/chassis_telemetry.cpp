/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis_telemetry.hpp"

#include <cmath>
#include <limits>

namespace dust_control::ros {
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
    const auto& lqr14 = sample.lqr14;
    message.lqr14_status = connected ? static_cast<uint8_t>(lqr14.status) : 0;
    message.lqr14_blend = connected ? lqr14.blend : 0.0;
    message.hip_height_valid =
        message.feedback_valid && message.imu_fresh && message.left_leg_fresh &&
        message.right_leg_fresh && input.hip_height_valid;
    message.hip_height_m = number(input.hip_height_m, message.hip_height_valid);
    message.left_hip_height_m =
        number(input.hip_height_side_m.left, message.hip_height_valid);
    message.right_hip_height_m =
        number(input.hip_height_side_m.right, message.hip_height_valid);
    message.hip_height_rate_m_s =
        number(input.hip_height_rate_m_s, message.hip_height_valid);
    const bool reference_valid =
        message.hip_height_valid && lqr14.height_valid &&
        (lqr14.status == chassis::Lqr14Status::kActive ||
         lqr14.status == chassis::Lqr14Status::kShadow ||
         lqr14.status == chassis::Lqr14Status::kWaitingSupport);
    message.hip_height_reference_m =
        number(lqr14.height_reference_m, reference_valid);
    message.hip_height_rate_reference_m_s =
        number(lqr14.height_rate_reference_m_s, reference_valid);
    message.flat_ground_mismatch_m =
        number(input.flat_ground_mismatch_m, message.hip_height_valid);
    const bool support_valid =
        message.feedback_valid && input.support_force_valid;
    message.left_support_force_n =
        number(input.measured_support_force_n.left, support_valid);
    message.right_support_force_n =
        number(input.measured_support_force_n.right, support_valid);
    const bool force_valid = message.hip_height_valid &&
                             (lqr14.status == chassis::Lqr14Status::kActive ||
                              lqr14.status == chassis::Lqr14Status::kShadow);
    message.lqr14_left_force_target_n =
        number(lqr14.requested_vertical_force_n.left, force_valid);
    message.lqr14_right_force_target_n =
        number(lqr14.requested_vertical_force_n.right, force_valid);
    message.lqr14_left_force_allocated_n =
        number(lqr14.allocated_vertical_force_n.left, force_valid);
    message.lqr14_right_force_allocated_n =
        number(lqr14.allocated_vertical_force_n.right, force_valid);
    const auto& motion = sample.motion;
    const bool motion_valid = message.feedback_valid && message.imu_fresh && motion.valid;
    message.motion_diagnostics_valid = motion_valid;
    message.raw_body_speed_mps = number(motion.raw_speed, motion_valid);
    message.estimated_body_speed_mps = number(motion.estimated_speed, motion_valid);
    message.target_body_speed_mps = number(motion.target_speed, motion_valid);
    message.estimated_body_position_m = number(motion.estimated_position, motion_valid);
    message.target_body_position_m = number(motion.target_position, motion_valid);
    message.body_position_error_m = number(motion.position_error, motion_valid);
    message.body_speed_error_mps = number(motion.speed_error, motion_valid);
    message.pitch_reference_deg = number(motion.pitch_reference * (180.0 / 3.14159265358979323846), motion_valid);
    message.theta_reference_deg = number(motion.theta_reference * (180.0 / 3.14159265358979323846), motion_valid);
    message.roll_reference_deg = number(motion.roll_reference * (180.0 / 3.14159265358979323846), motion_valid);
    message.remote_chassis_x = number(input.remote.chassis_x, message.remote_fresh);
    return message;
}
}  // namespace dust_control::ros
