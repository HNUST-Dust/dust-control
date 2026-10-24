/* SPDX-License-Identifier: Apache-2.0 */
#include "motor_config.hpp"

#include <cmath>

#include "hardware/config_yaml.hpp"

namespace dust::devices {
namespace {
double FiniteNumber(const YAML::Node& node) {
    const double value = node.as<double>();
    if (!std::isfinite(value)) {
        throw std::runtime_error("Motor parameter must be finite");
    }
    return value;
}
}  // namespace

void ValidateMotorConfig(const MotorConfig& config) {
    std::set<std::pair<unsigned, unsigned>> feedback;
    std::set<std::pair<unsigned, unsigned>> commands;
    for (size_t i = 0; i < config.addresses.size(); ++i) {
        const auto& address = config.addresses[i];
        if (address.bus != 2 && address.bus != 3) {
            throw std::runtime_error("Gateway supports only CAN2/CAN3");
        }
        if (address.command_id > 0x7ff || address.feedback_id > 0x7ff ||
            (i < 2 &&
             (address.command_slot > 3 ||
              address.command_id != platform::motor_model::kDjiCommandId ||
              address.feedback_id != platform::motor_model::DjiFeedbackId(
                                         address.command_slot)))) {
            throw std::runtime_error(
                "Invalid motor CAN address or DJI slot mismatch");
        }
        if (!feedback.insert({address.bus, address.feedback_id}).second ||
            !commands.insert({address.bus, address.command_id}).second) {
            throw std::runtime_error("Duplicate motor CAN address");
        }
        if (!config.freshness_us[i] ||
            config.freshness_us[i] > UINT32_MAX - 1ULL) {
            throw std::runtime_error("Invalid motor freshness deadline");
        }
    }
    for (const double sign : config.wheel_command_sign) {
        if (sign != -1 && sign != 1) {
            throw std::runtime_error("Motor command_sign must be -1 or 1");
        }
    }
    if (!std::isfinite(config.dji_current_per_nm) ||
        config.dji_current_per_nm <= 0 ||
        !std::isfinite(config.wheel_torque_limit_nm())) {
        throw std::runtime_error("Invalid DJI current conversion or limit");
    }
}

MotorConfig LoadMotorConfig(const std::string& file) {
    using namespace dust::config_yaml;
    MotorConfig candidate;
    const std::array<std::string, 6> names = {"left_wheel",    "right_wheel",
                                              "left_joint_b",  "left_joint_d",
                                              "right_joint_b", "right_joint_d"};
    const auto root = YAML::LoadFile(file);
    auto keys = std::set<std::string>(names.begin(), names.end());
    keys.insert({"schema_version", "dji"});
    ValidateKeys(root, keys);
    if (ParseUnsignedInteger(root["schema_version"], 3) != 3) {
        throw std::runtime_error("Unsupported motors schema version");
    }
    for (size_t i = 0; i < names.size(); ++i) {
        const auto motor = root[names[i]];
        auto expected = std::set<std::string>{"bus", "freshness_deadline_us"};
        if (i < 2) {
            expected.insert({"command_slot", "command_sign"});
        } else {
            expected.insert({"command_id", "feedback_id"});
        }
        ValidateKeys(motor, expected);
        auto& address = candidate.addresses[i];
        address.bus = ParseUnsignedInteger(motor["bus"], 3);
        address.command_id =
            i < 2 ? platform::motor_model::kDjiCommandId
                  : ParseUnsignedInteger(motor["command_id"], 0x7ff);

        address.command_slot =
            i < 2 ? ParseUnsignedInteger(motor["command_slot"], 3) : 0;
        address.feedback_id =
            i < 2 ? platform::motor_model::DjiFeedbackId(address.command_slot)
                  : ParseUnsignedInteger(motor["feedback_id"], 0x7ff);
        candidate.freshness_us[i] = ParseFreshnessDeadline(
            motor["freshness_deadline_us"], UINT32_MAX - 1ULL);
        if (i < 2) {
            candidate.wheel_command_sign[i] =
                FiniteNumber(motor["command_sign"]);
        }
    }
    ValidateKeys(root["dji"], {"current_per_nm"});
    candidate.dji_current_per_nm = FiniteNumber(root["dji"]["current_per_nm"]);
    ValidateMotorConfig(candidate);
    return candidate;
}
}  // namespace dust::devices
