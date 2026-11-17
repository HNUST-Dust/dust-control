/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis_telemetry.hpp"

#include <algorithm>
#include <cmath>
#include "hardware/motors/motor_config.hpp"
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
    const auto& lqr = sample.lqr;
    message.lqr_endpoint_gain = connected && lqr.endpoint_gain;
    message.lqr_height_control_active = connected && lqr.height_control_active;
    message.lqr_status = connected ? static_cast<uint8_t>(lqr.status) : 0;
    message.lqr_blend = connected ? lqr.blend : 0.0;
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
        message.hip_height_valid && lqr.height_valid &&
        (lqr.status == chassis::LqrStatus::kActive ||
         lqr.status == chassis::LqrStatus::kWaitingSupport);
    message.hip_height_reference_m =
        number(lqr.height_reference_m, reference_valid);
    message.hip_height_rate_reference_m_s =
        number(lqr.height_rate_reference_m_s, reference_valid);
    message.flat_ground_mismatch_m =
        number(input.flat_ground_mismatch_m, message.hip_height_valid);
    const bool support_valid =
        message.feedback_valid && input.support_force_valid;
    message.left_support_force_n =
        number(input.measured_support_force_n.left, support_valid);
    message.right_support_force_n =
        number(input.measured_support_force_n.right, support_valid);
    const bool force_valid = message.hip_height_valid &&
                             lqr.status == chassis::LqrStatus::kActive;
    message.lqr_left_force_target_n =
        number(lqr.requested_vertical_force_n.left, force_valid);
    message.lqr_right_force_target_n =
        number(lqr.requested_vertical_force_n.right, force_valid);
    message.lqr_left_force_allocated_n =
        number(lqr.allocated_vertical_force_n.left, force_valid);
    message.lqr_right_force_allocated_n =
        number(lqr.allocated_vertical_force_n.right, force_valid);
    const bool wheel_valid = message.feedback_valid && message.imu_fresh &&
        (lqr.status == chassis::LqrStatus::kActive ||
         lqr.status == chassis::LqrStatus::kDegraded || lqr.status == chassis::LqrStatus::kAction ||
         lqr.status == chassis::LqrStatus::kExternalSupport);
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
    const auto& estimator = motion.estimation;
    const bool estimator_valid = motion_valid && estimator.valid;
    message.motion_estimator_valid = estimator_valid;
    message.motion_acceleration_used =
        estimator_valid && estimator.acceleration_used;
    message.motion_wheel_speed_used =
        estimator_valid && estimator.wheel_speed_used;
    message.body_acceleration_mps2 =
        number(motion.acceleration_mps2, estimator_valid);
    message.body_acceleration_bias_mps2 =
        number(motion.acceleration_bias_mps2, estimator_valid);
    message.motion_wheel_speed_variance =
        number(estimator.wheel_speed_variance,
               estimator_valid && estimator.wheel_speed_used);
    for (size_t i = 0; i < 3; ++i) {
        message.motion_covariance_diagonal[i] =
            number(estimator.covariance_diagonal[i], estimator_valid);
    }
    for (size_t i = 0; i < 2; ++i) {
        const auto& wheel = estimator.wheels[i];
        message.wheel_speed_status[i] =
            estimator_valid ? static_cast<uint8_t>(wheel.status) : 0;
        message.wheel_speed_confidence[i] =
            number(wheel.confidence, estimator_valid);
        message.wheel_speed_prediction_residual_mps[i] =
            number(wheel.prediction_residual_mps, estimator_valid);
        message.wheel_speed_pair_residual_mps[i] =
            number(wheel.pair_residual_mps, estimator_valid);
    }
    message.control_dt_s = number(input.dt, connected);
    message.imu_sequence = input.imu.sequence;
    message.imu_sample_monotonic_us = input.imu.precise_timestamp_us;
    for (size_t i = 0; i < 3; ++i) {
        message.imu_raw_accel_g[i] =
            number(input.imu.accel_g[i], message.imu_fresh);
    }
    message.roll_rate_rad_s = number(input.roll_rate, message.imu_fresh);
    message.yaw_rate_rad_s =
        number(input.imu.yaw_rate_rad_s, message.imu_fresh);
    message.forward_acceleration_valid =
        connected && input.forward_acceleration_valid;
    message.forward_acceleration_mps2 = number(
        input.forward_acceleration_mps2, message.forward_acceleration_valid);
    message.motion_acceleration_new_sample =
        estimator_valid && estimator.acceleration_new_sample;
    message.motion_yaw_valid = estimator_valid && estimator.yaw_valid;
    message.motion_support_valid = estimator_valid && estimator.support_valid;
    message.motion_prior_speed_mps =
        number(estimator.prior_speed_mps, estimator_valid);
    message.motion_prior_speed_variance =
        number(estimator.prior_speed_variance, estimator_valid);
    message.motion_fused_wheel_speed_mps =
        number(estimator.fused_wheel_speed_mps,
               estimator_valid && estimator.wheel_speed_used);
    for (size_t i = 0; i < 2; ++i) {
        const auto& wheel = estimator.wheels[i];
        message.motion_wheel_valid[i] =
            estimator_valid && estimator.wheel_valid[i];
        message.motion_center_speed_mps[i] = number(
            estimator.center_speed_mps[i], message.motion_wheel_valid[i]);
        message.motion_support_fraction[i] =
            number(estimator.support_fraction[i], message.motion_support_valid);
        message.wheel_pair_comparison_valid[i] =
            estimator_valid && wheel.pair_comparison_valid;
        message.wheel_prediction_gate_active[i] =
            estimator_valid && wheel.prediction_gate_active;
        message.wheel_contact_known[i] = estimator_valid && wheel.contact_known;
        message.wheel_contact[i] = estimator_valid && wheel.contact;
        message.wheel_low_support_time_s[i] =
            number(wheel.low_support_time_s, message.wheel_contact_known[i]);
        message.wheel_prediction_gate_mps[i] = number(
            wheel.prediction_gate_mps, message.wheel_prediction_gate_active[i]);
        message.wheel_prediction_ratio[i] = number(
            wheel.prediction_ratio, message.wheel_prediction_gate_active[i] &&
                                        message.motion_wheel_valid[i]);
        message.wheel_speed_evidence[i] = estimator_valid ? wheel.evidence : 0;
        // Do not encode an unavailable comparison as a measured zero residual.
        message.wheel_speed_pair_residual_mps[i] = number(
            wheel.pair_residual_mps, message.wheel_pair_comparison_valid[i]);
        message.wheel_speed_prediction_residual_mps[i] = number(
            wheel.prediction_residual_mps, message.motion_wheel_valid[i]);
    }
    const auto& filter = estimator.parameters;
    const auto& thresholds = filter.contact;
    const std::array<double, 4> filter_parameters = {
        filter.acceleration_process_noise, filter.bias_random_walk_psd,
        filter.wheel_speed_variance, filter.acceleration_variance};
    const std::array<double, 3> contact_parameters = {
        thresholds.contact_off_fraction, thresholds.contact_on_fraction,
        thresholds.contact_loss_delay_s};
    for (size_t i = 0; i < filter_parameters.size(); ++i) {
        message.motion_filter_parameters[i] =
            number(filter_parameters[i], estimator_valid);
    }
    for (size_t i = 0; i < contact_parameters.size(); ++i) {
        message.motion_contact_parameters[i] =
            number(contact_parameters[i], estimator_valid);
    }
    // Historical adaptive-noise parameters no longer apply. Keep the wire
    // field explicitly unavailable rather than advertise inactive thresholds.
    message.motion_reliability_parameters.fill(invalid);
    message.remote_chassis_x = number(input.remote.chassis_x, message.remote_fresh);
    message.body_position_error_unclamped_m = number(motion.unclamped_position_error, motion_valid);
    message.left_theta_deg = number(input.theta.left * (180.0 / 3.14159265358979323846), motion_valid);
    message.right_theta_deg = number(input.theta.right * (180.0 / 3.14159265358979323846), motion_valid);
    for (int j = 0; j < 14; ++j) {
        message.lqr_left_wheel_contribution_nm[j] = number(lqr.left_wheel_contribution[j], wheel_valid);
        message.lqr_right_wheel_contribution_nm[j] = number(lqr.right_wheel_contribution[j], wheel_valid);
    }
    message.lqr_left_wheel_requested_nm = number(lqr.requested_wheel_torque.left, wheel_valid);
    message.lqr_left_wheel_allocated_nm = number(lqr.allocated_wheel_torque.left, wheel_valid);
    message.left_wheel_output_command_nm = number(lqr.output_wheel_torque.left, wheel_valid);
    message.lqr_right_wheel_requested_nm = number(lqr.requested_wheel_torque.right, wheel_valid);
    message.lqr_right_wheel_allocated_nm = number(lqr.allocated_wheel_torque.right, wheel_valid);
    message.right_wheel_output_command_nm = number(lqr.output_wheel_torque.right, wheel_valid);
    for (size_t i = 0; i < 6; ++i) {
        message.motor_target_mode[i] = connected ? static_cast<uint8_t>(sample.motor_targets[i].mode) : 0;
        message.motor_feedback_age_us[i] = number(double(input.motor_feedback_age_us[i]),
            connected && input.motor_feedback_age_us[i] != UINT32_MAX);
    }
    for (size_t i = 0; i < 2; ++i) {
        const auto side = i == 0 ? chassis::Side::kLeft : chassis::Side::kRight;
        const bool fresh = connected && !(input.motor_stale_mask & (1U << i));
        const auto& target = sample.motor_targets[i];
        const bool torque_mode = connected && target.mode == devices::MotorMode::kTorque;
        message.wheel_current_feedback_raw[i] = number(input.wheel_current_feedback_raw.Get(side), fresh);
        message.wheel_speed_feedback_rpm[i] = number(input.wheel_motor_speed_rpm.Get(side), fresh);
        message.wheel_final_command_nm[i] = number(target.torque_nm, torque_mode);
        const double current = sample.wheel_command_sign[i] * target.torque_nm * devices::MotorConfig::dji_current_per_nm;
        const double quantized = std::isfinite(current) ? double(static_cast<int16_t>(std::clamp(current,
            -double(devices::MotorConfig::dji_protocol_current_limit), double(devices::MotorConfig::dji_protocol_current_limit)))) : 0.;
        message.wheel_current_command_raw[i] = number(quantized, torque_mode);
        message.theta_rate_rad_s[i] = number(input.theta_rate.Get(side), motion_valid);
        for (size_t j = 0; j < 2; ++j) {
            const auto joint = j == 0 ? chassis::Joint::kB : chassis::Joint::kD;
            const size_t slot = 2 + i*2 + j, index = i*2+j;
            const auto& command = sample.motor_targets[slot];
            const bool joint_fresh = connected && !(input.motor_stale_mask & (1U << slot));
            message.joint_final_command_nm[index] = number(command.torque_nm,
                connected && command.mode == devices::MotorMode::kTorque);
            message.joint_torque_feedback_nm[index] = number(input.joint_torque_feedback.Get(side).Get(joint), joint_fresh);
            message.joint_position_feedback_rad[index] = number(input.joint_position.Get(side).Get(joint), joint_fresh);
            message.joint_velocity_feedback_rad_s[index] = number(input.joint_velocity.Get(side).Get(joint), joint_fresh);
        }
    }
    message.pitch_rate_rad_s = number(input.pitch_rate, message.imu_fresh);
    return message;
}
}  // namespace dust_control::ros
