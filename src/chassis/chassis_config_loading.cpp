/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis_config.h"
#include "yaml_validation.hpp"
namespace dust_control::modules {
ChassisConfig LoadChassisConfig(const std::string& file) {
    using dust_control::yaml_validation::ValidateKeys;
    const auto root = YAML::LoadFile(file);
    ValidateKeys(root,
                 {"actuator", "balance", "flight", "gas_spring", "hardware",
                  "jump", "mechanics", "safety", "schema_version", "stool",
                  "support_force_estimator", "vmc"});
    ValidateKeys(root["mechanics"],
                 {"half_wheel_track_m", "joint_b_zero_offset_rad",
                  "joint_d_zero_offset_rad", "leg_geometry_m", "robot_mass_kg",
                  "wheel_radius_m", "wheel_reduction"});
    ValidateKeys(root["mechanics"]["leg_geometry_m"],
                 {"ab", "ad", "ag", "bc", "cd", "gh"});
    ValidateKeys(
        root["balance"],
        {"body_acceleration_limit_mps2", "max_body_speed_mps",
         "max_leg_length_m", "max_turn_wheel_speed_mps", "min_leg_length_m",
         "remote_deadband", "roll_compensation_force_limit_n",
         "roll_compensation_kp_n_per_rad", "theta_bias_deg",
         "turn_acceleration_limit_mps2", "turn_torque_limit_nm"});
    ValidateKeys(root["stool"],
                 {"joint_position_kp", "joint_position_max_speed_rad_s",
                  "joint_speed_kp", "target_leg_length_m"});
    ValidateKeys(root["gas_spring"],
                 {"left_spring_force_n", "left_spring_upper_offset_m",
                  "right_spring_force_n", "right_spring_upper_offset_m",
                  "spring_lower_distance_m"});
    ValidateKeys(root["support_force_estimator"], {"wheel_mass_kg"});
    ValidateKeys(root["flight"], {"airborne_push_force_n"});
    ValidateKeys(root["jump"], {"thrust_torque_nm"});
    ValidateKeys(root["vmc"],
                 {"force_limit_n", "integral_force_limit_n", "leg_length_kd",
                  "leg_length_ki", "leg_length_kp"});
    ValidateKeys(root["actuator"], {"joint_torque_limit_nm"});
    ValidateKeys(root["safety"], {"tilt_cutoff_deg"});
    ValidateKeys(root["hardware"], {"left", "right"});
    ValidateKeys(root["hardware"]["left"], {"leg_angle_offset_rad"});
    ValidateKeys(root["hardware"]["right"], {"leg_angle_offset_rad"});
    if (dust_control::yaml_validation::ParseUnsignedInteger(
            root["schema_version"], 2) != 2) {
        throw std::runtime_error("Unsupported chassis schema version");
    }
    ChassisConfig config;
    config.joint_b_zero_offset =
        root["mechanics"]["joint_b_zero_offset_rad"].as<double>();
    config.joint_d_zero_offset =
        root["mechanics"]["joint_d_zero_offset_rad"].as<double>();
    config.wheel_reduction = root["mechanics"]["wheel_reduction"].as<double>();
    config.wheel_radius = root["mechanics"]["wheel_radius_m"].as<double>();
    config.half_wheel_track =
        root["mechanics"]["half_wheel_track_m"].as<double>();
    config.robot_mass = root["mechanics"]["robot_mass_kg"].as<double>();
    config.theta_balance_bias_deg =
        root["balance"]["theta_bias_deg"].as<double>();
    config.remote_deadband = root["balance"]["remote_deadband"].as<double>();
    config.min_commanded_leg_length =
        root["balance"]["min_leg_length_m"].as<double>();
    config.max_commanded_leg_length =
        root["balance"]["max_leg_length_m"].as<double>();
    config.max_body_speed = root["balance"]["max_body_speed_mps"].as<double>();
    config.body_acceleration_limit =
        root["balance"]["body_acceleration_limit_mps2"].as<double>();
    config.max_turn_wheel_speed =
        root["balance"]["max_turn_wheel_speed_mps"].as<double>();
    config.turn_acceleration_limit =
        root["balance"]["turn_acceleration_limit_mps2"].as<double>();
    config.turn_torque_limit =
        root["balance"]["turn_torque_limit_nm"].as<double>();
    config.roll_compensation_kp =
        root["balance"]["roll_compensation_kp_n_per_rad"].as<double>();
    config.roll_compensation_force_limit =
        root["balance"]["roll_compensation_force_limit_n"].as<double>();
    config.joint_torque_limit =
        root["actuator"]["joint_torque_limit_nm"].as<double>();
    config.tilt_cutoff_deg = root["safety"]["tilt_cutoff_deg"].as<double>();
    config.length_ab = root["mechanics"]["leg_geometry_m"]["ab"].as<double>();
    config.length_bc = root["mechanics"]["leg_geometry_m"]["bc"].as<double>();
    config.length_cd = root["mechanics"]["leg_geometry_m"]["cd"].as<double>();
    config.length_ad = root["mechanics"]["leg_geometry_m"]["ad"].as<double>();
    config.length_ag = root["mechanics"]["leg_geometry_m"]["ag"].as<double>();
    config.length_gh = root["mechanics"]["leg_geometry_m"]["gh"].as<double>();
    config.stool_target_leg_length =
        root["stool"]["target_leg_length_m"].as<double>();
    config.joint_position_kp = root["stool"]["joint_position_kp"].as<double>();
    config.joint_position_max_speed =
        root["stool"]["joint_position_max_speed_rad_s"].as<double>();
    config.joint_speed_kp = root["stool"]["joint_speed_kp"].as<double>();
    config.gas_spring_left_force =
        root["gas_spring"]["left_spring_force_n"].as<double>();
    config.gas_spring_right_force =
        root["gas_spring"]["right_spring_force_n"].as<double>();
    config.gas_spring_left_upper_offset =
        root["gas_spring"]["left_spring_upper_offset_m"].as<double>();
    config.gas_spring_right_upper_offset =
        root["gas_spring"]["right_spring_upper_offset_m"].as<double>();
    config.gas_spring_lower_distance =
        root["gas_spring"]["spring_lower_distance_m"].as<double>();
    config.support_wheel_mass =
        root["support_force_estimator"]["wheel_mass_kg"].as<double>();
    config.flight_airborne_push_force =
        root["flight"]["airborne_push_force_n"].as<double>();
    config.jump_thrust_torque = root["jump"]["thrust_torque_nm"].as<double>();
    config.leg_length_kp = root["vmc"]["leg_length_kp"].as<double>();
    config.leg_length_ki = root["vmc"]["leg_length_ki"].as<double>();
    config.leg_length_kd = root["vmc"]["leg_length_kd"].as<double>();
    config.integral_force_limit =
        root["vmc"]["integral_force_limit_n"].as<double>();
    config.force_limit = root["vmc"]["force_limit_n"].as<double>();
    config.left_leg_angle_offset =
        root["hardware"]["left"]["leg_angle_offset_rad"].as<double>();
    config.right_leg_angle_offset =
        root["hardware"]["right"]["leg_angle_offset_rad"].as<double>();
    ValidateChassisConfig(config);
    return config;
}
}  // namespace dust_control::modules
