/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis_config.h"

#include <cmath>
#include <stdexcept>
namespace dust_control::modules {
void ValidateChassisConfig(const ChassisConfig& config) {
    if (!std::isfinite(config.joint_b_zero_offset)) {
        throw std::runtime_error(
            "Invalid chassis parameter: mechanics.joint_b_zero_offset_rad");
    }
    if (!std::isfinite(config.joint_d_zero_offset)) {
        throw std::runtime_error(
            "Invalid chassis parameter: mechanics.joint_d_zero_offset_rad");
    }
    if (!std::isfinite(config.wheel_reduction) ||
        config.wheel_reduction <= 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: mechanics.wheel_reduction");
    }
    if (!std::isfinite(config.wheel_radius) || config.wheel_radius <= 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: mechanics.wheel_radius_m");
    }
    if (!std::isfinite(config.half_wheel_track) ||
        config.half_wheel_track <= 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: mechanics.half_wheel_track_m");
    }
    if (!std::isfinite(config.robot_mass) || config.robot_mass <= 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: mechanics.robot_mass_kg");
    }
    if (!std::isfinite(config.theta_balance_bias_deg) ||
        config.theta_balance_bias_deg < -90.0 ||
        config.theta_balance_bias_deg > 90.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: balance.theta_bias_deg");
    }
    if (!std::isfinite(config.remote_deadband) ||
        config.remote_deadband < 0.0 || config.remote_deadband > 0.99) {
        throw std::runtime_error(
            "Invalid chassis parameter: balance.remote_deadband");
    }
    if (!std::isfinite(config.min_commanded_leg_length) ||
        config.min_commanded_leg_length <= 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: balance.min_leg_length_m");
    }
    if (!std::isfinite(config.max_commanded_leg_length) ||
        config.max_commanded_leg_length <= 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: balance.max_leg_length_m");
    }
    if (!std::isfinite(config.max_body_speed) || config.max_body_speed <= 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: balance.max_body_speed_mps");
    }
    if (!std::isfinite(config.body_acceleration_limit) ||
        config.body_acceleration_limit < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: balance.body_acceleration_limit_mps2");
    }
    if (!std::isfinite(config.max_turn_wheel_speed) ||
        config.max_turn_wheel_speed <= 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: balance.max_turn_wheel_speed_mps");
    }
    if (!std::isfinite(config.turn_acceleration_limit) ||
        config.turn_acceleration_limit < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: balance.turn_acceleration_limit_mps2");
    }
    if (!std::isfinite(config.turn_torque_limit) ||
        config.turn_torque_limit < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: balance.turn_torque_limit_nm");
    }
    if (!std::isfinite(config.roll_compensation_kp) ||
        config.roll_compensation_kp < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: "
            "balance.roll_compensation_kp_n_per_rad");
    }
    if (!std::isfinite(config.roll_compensation_force_limit) ||
        config.roll_compensation_force_limit < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: "
            "balance.roll_compensation_force_limit_n");
    }
    if (!std::isfinite(config.joint_torque_limit) ||
        config.joint_torque_limit < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: actuator.joint_torque_limit_nm");
    }
    if (!std::isfinite(config.tilt_cutoff_deg) ||
        config.tilt_cutoff_deg < 0.0 || config.tilt_cutoff_deg > 180.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: safety.tilt_cutoff_deg");
    }
    if (!std::isfinite(config.length_ab) || config.length_ab <= 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: mechanics.leg_geometry_m.ab");
    }
    if (!std::isfinite(config.length_bc) || config.length_bc <= 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: mechanics.leg_geometry_m.bc");
    }
    if (!std::isfinite(config.length_cd) || config.length_cd <= 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: mechanics.leg_geometry_m.cd");
    }
    if (!std::isfinite(config.length_ad) || config.length_ad <= 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: mechanics.leg_geometry_m.ad");
    }
    if (!std::isfinite(config.length_ag) || config.length_ag <= 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: mechanics.leg_geometry_m.ag");
    }
    if (!std::isfinite(config.length_gh) || config.length_gh <= 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: mechanics.leg_geometry_m.gh");
    }
    if (!std::isfinite(config.stool_target_leg_length) ||
        config.stool_target_leg_length <= 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: stool.target_leg_length_m");
    }
    if (!std::isfinite(config.joint_position_kp) ||
        config.joint_position_kp < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: stool.joint_position_kp");
    }
    if (!std::isfinite(config.joint_position_max_speed) ||
        config.joint_position_max_speed <= 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: stool.joint_position_max_speed_rad_s");
    }
    if (!std::isfinite(config.joint_speed_kp) || config.joint_speed_kp < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: stool.joint_speed_kp");
    }
    if (!std::isfinite(config.gas_spring_left_force) ||
        config.gas_spring_left_force < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: gas_spring.left_spring_force_n");
    }
    if (!std::isfinite(config.gas_spring_right_force) ||
        config.gas_spring_right_force < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: gas_spring.right_spring_force_n");
    }
    if (!std::isfinite(config.gas_spring_left_upper_offset)) {
        throw std::runtime_error(
            "Invalid chassis parameter: gas_spring.left_spring_upper_offset_m");
    }
    if (!std::isfinite(config.gas_spring_right_upper_offset)) {
        throw std::runtime_error(
            "Invalid chassis parameter: "
            "gas_spring.right_spring_upper_offset_m");
    }
    if (!std::isfinite(config.gas_spring_lower_distance) ||
        config.gas_spring_lower_distance < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: gas_spring.spring_lower_distance_m");
    }
    if (!std::isfinite(config.support_wheel_mass) ||
        config.support_wheel_mass < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: support_force_estimator.wheel_mass_kg");
    }
    if (!std::isfinite(config.flight_airborne_push_force) ||
        config.flight_airborne_push_force < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: flight.airborne_push_force_n");
    }
    if (!std::isfinite(config.jump_thrust_torque) ||
        config.jump_thrust_torque < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: jump.thrust_torque_nm");
    }
    if (!std::isfinite(config.leg_length_kp) || config.leg_length_kp < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: vmc.leg_length_kp");
    }
    if (!std::isfinite(config.leg_length_ki) || config.leg_length_ki < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: vmc.leg_length_ki");
    }
    if (!std::isfinite(config.leg_length_kd) || config.leg_length_kd < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: vmc.leg_length_kd");
    }
    if (!std::isfinite(config.integral_force_limit) ||
        config.integral_force_limit < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: vmc.integral_force_limit_n");
    }
    if (!std::isfinite(config.force_limit) || config.force_limit < 0.0) {
        throw std::runtime_error(
            "Invalid chassis parameter: vmc.force_limit_n");
    }
    if (!std::isfinite(config.left_leg_angle_offset)) {
        throw std::runtime_error(
            "Invalid chassis parameter: hardware.left.leg_angle_offset_rad");
    }
    if (!std::isfinite(config.right_leg_angle_offset)) {
        throw std::runtime_error(
            "Invalid chassis parameter: hardware.right.leg_angle_offset_rad");
    }
    if (config.min_commanded_leg_length >= config.max_commanded_leg_length ||
        config.stool_target_leg_length < config.min_commanded_leg_length ||
        config.stool_target_leg_length > config.max_commanded_leg_length ||
        config.gas_spring_lower_distance > config.length_gh ||
        !std::isfinite(config.max_yaw_rate()) ||
        !std::isfinite(config.yaw_acceleration_limit())) {
        throw std::runtime_error(
            "Inconsistent chassis dimensions or operating limits");
    }
}
}  // namespace dust_control::modules
