/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis_telemetry.hpp"

#include <cassert>
#include <cmath>

int main() {
    dust_control::chassis::ChassisObservation sample;
    sample.sequence = 17;
    sample.monotonic_ns = 1000000000;
    sample.control_duration_ns = 12500;
    sample.input.imu_fresh = true;
    sample.input.wheel_current_feedback_raw = {-80, 90};
    sample.wheel_command_sign = {-1, 1};
    sample.motor_targets[0] = {dust_control::devices::MotorMode::kTorque, .1};
    sample.motor_targets[2] = {dust_control::devices::MotorMode::kTorque, 3.};
    sample.input.joint_torque_feedback.left.b = 2.8;
    sample.input.imu.pitch_deg = 12.5;
    sample.input.imu_age_us = 123;
    sample.input.leg.left.length = .25;
    sample.input.leg.left.hz = -.23;
    sample.input.leg.right.length = .26;
    const rclcpp::Time now(2000000000LL);
    auto message =
        dust_control::ros::MakeChassisTelemetry(sample, now, 1010000000);
    assert(message.sequence == 17 && message.version == 7);
    assert(message.header.stamp.sec == 1 &&
           message.header.stamp.nanosec == 990000000);
    assert(message.pitch_deg == 12.5 && message.imu_age_us == 123);
    assert(message.wheel_current_feedback_raw[0] == -80);
    assert(message.wheel_current_command_raw[0] == -345);
    assert(message.joint_final_command_nm[0] == 3.);
    assert(message.joint_torque_feedback_nm[0] == 2.8);
    assert(std::isnan(message.wheel_final_command_nm[1]));
    assert(message.left_length_m == .25 && message.left_height_m == .23);
    assert(message.transport_age_ms == 10 &&
           message.control_duration_us == 12.5);
    assert(!message.motion_diagnostics_valid && std::isnan(message.raw_body_speed_mps));
    sample.motion.valid = true;
    sample.motion.raw_speed = .02;
    sample.motion.estimated_speed = .01;
    sample.motion.position_error = .15;
    sample.motion.unclamped_position_error = .4;
    sample.motion.roll_reference = .1;
    sample.input.feedback_valid = true;
    sample.input.support_force_valid = true;
    sample.input.measured_support_force_n = {50, 60};
    sample.lqr.height_valid = true;
    sample.lqr.height_m = .3;
    sample.input.hip_height_valid = true;
    sample.input.hip_height_m = .3;
    sample.lqr.status = dust_control::chassis::LqrStatus::kActive;
    sample.lqr.requested_vertical_force_n = {55, 65};
    sample.lqr.left_wheel_contribution[0] = .25;
    message = dust_control::ros::MakeChassisTelemetry(sample, now, 1010000000);
    assert(message.motion_diagnostics_valid && message.raw_body_speed_mps == .02);
    assert(message.estimated_body_speed_mps == .01 && message.body_position_error_m == .15);
    assert(std::abs(message.roll_reference_deg - 5.7295779513) < 1e-9);
    assert(message.hip_height_valid && message.hip_height_m == .3);
    assert(message.lqr_status == 2 && message.left_support_force_n == 50);
    assert(message.lqr_right_force_target_n == 65);
    assert(message.body_position_error_unclamped_m == .4);
    assert(message.lqr_left_wheel_contribution_nm[0] == .25);
    sample.lqr.status = dust_control::chassis::LqrStatus::kDegraded;
    sample.lqr.endpoint_gain = true;
    sample.input.hip_height_valid = false;
    message = dust_control::ros::MakeChassisTelemetry(sample, now, 1010000000);
    assert(message.lqr_status == 9 && message.lqr_endpoint_gain);
    assert(message.lqr_left_wheel_contribution_nm[0] == .25);
    assert(std::isnan(message.lqr_left_force_target_n));
    sample.input.motor_stale_mask = 0x0c;
    sample.input.imu_fresh = false;
    message = dust_control::ros::MakeChassisTelemetry(sample, now, 1010000000);
    assert(std::isnan(message.pitch_deg));
    assert(std::isnan(message.lqr_left_wheel_contribution_nm[0]));
    assert(!message.motion_diagnostics_valid && std::isnan(message.estimated_body_speed_mps));
    assert(!message.left_leg_fresh && std::isnan(message.left_length_m));
    assert(message.right_leg_fresh && message.right_length_m == .26);
    sample.input.feedback_invalid_mask = 8;
    message = dust_control::ros::MakeChassisTelemetry(sample, now, 1010000000);
    assert(!message.right_leg_fresh && std::isnan(message.right_length_m));
    message = dust_control::ros::MakeChassisTelemetry(sample, now, 1300000000);
    assert(!message.source_connected && !message.imu_fresh);
    assert(message.motor_stale_mask == 0x3f && std::isnan(message.imu_age_us));
    assert(!message.right_leg_fresh && !message.left_leg_fresh);
    assert(!message.hip_height_valid && std::isnan(message.hip_height_m));
    assert(std::isnan(message.lqr_right_force_target_n));
    assert(std::isnan(message.wheel_current_feedback_raw[0]));
    assert(std::isnan(message.joint_final_command_nm[0]));
}
