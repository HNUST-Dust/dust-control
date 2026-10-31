/* SPDX-License-Identifier: Apache-2.0 */
#include "motor_config.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>

#include "yaml_validation.hpp"

namespace dust_control::devices {
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
    std::set<std::tuple<std::string, unsigned, unsigned>> feedback;
    std::set<std::tuple<std::string, unsigned, unsigned>> commands;
    for (size_t i = 0; i < config.addresses.size(); ++i) {
        if (!config.present[i]) {
            continue;
        }
        const auto& adapter = config.adapters[i];
        if (adapter.empty()) {
            throw std::runtime_error("Motor adapter must be a nonempty name");
        }
        const auto& address = config.addresses[i];
        const auto required = i < 2 ? MotorProtocol::kDji : MotorProtocol::kDm;
        if (config.protocols[i] != required) {
            throw std::runtime_error(
                "Current chassis requires DJI wheels and DM joints");
        }
        if (address.bus != 0 && address.bus != 3) {
            throw std::runtime_error("Gateway supports only CAN0/CAN3");
        }
        if (address.command_id > 0x7ff || address.feedback_id > 0x7ff ||
            (i < 2 && (address.command_slot > 3 ||
                       address.command_id !=
                           dust_control::platform::motor_model::kDjiCommandId ||
                       address.feedback_id !=
                           dust_control::platform::motor_model::DjiFeedbackId(
                               address.command_slot)))) {
            throw std::runtime_error(
                "Invalid motor CAN address or DJI slot mismatch");
        }
        if (!feedback.insert({adapter, address.bus, address.feedback_id})
                 .second ||
            !commands.insert({adapter, address.bus, address.command_id})
                 .second) {
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
}

MotorConfig LoadMotorConfig(const std::string& file) {
    using namespace dust_control::yaml_validation;
    MotorConfig candidate;
    candidate.present.fill(false);
    const std::array<std::string, 6> names = {"left_wheel",    "right_wheel",
                                              "left_joint_b",  "left_joint_d",
                                              "right_joint_b", "right_joint_d"};
    const auto root = YAML::LoadFile(file);
    ValidateKeys(root, {"schema_version", "motors"});
    if (ParseUnsignedInteger(root["schema_version"], 5) != 5) {
        throw std::runtime_error(
            "Unsupported motors schema; use version 5 with adapter and bus on "
            "each motor");
    }
    const auto motors = root["motors"];
    if (!motors.IsSequence()) {
        throw std::runtime_error("motors must be a sequence");
    }
    std::set<std::string> seen;
    for (const auto& motor : motors) {
        const auto name = motor["name"].as<std::string>();
        const auto found = std::find(names.begin(), names.end(), name);
        if (found == names.end() || !seen.insert(name).second) {
            throw std::runtime_error("Unknown or duplicate motor name: " +
                                     name);
        }
        const size_t i = static_cast<size_t>(found - names.begin());
        candidate.present[i] = true;
        candidate.adapters[i] = motor["adapter"].as<std::string>();
        if (motor["adapter"].IsNull() || candidate.adapters[i].empty()) {
            throw std::runtime_error("Motor adapter must be a nonempty name: " +
                                     name);
        }
        const auto protocol = motor["protocol"].as<std::string>();
        if (protocol == "dji") {
            candidate.protocols[i] = MotorProtocol::kDji;
        } else if (protocol == "dm") {
            candidate.protocols[i] = MotorProtocol::kDm;
        } else {
            throw std::runtime_error("Unsupported motor protocol: " + protocol);
        }
        std::set<std::string> expected{"name",
                                       "adapter",
                                       "bus",
                                       "protocol",
                                       "command_id",
                                       "feedback_ids",
                                       "freshness_deadline_us"};
        if (candidate.protocols[i] == MotorProtocol::kDji) {
            expected.insert({"command_slot", "command_sign"});
        }
        ValidateKeys(motor, expected);
        const auto ids = motor["feedback_ids"];
        if (!ids.IsSequence() || ids.size() != 1) {
            throw std::runtime_error(
                "Current motor protocols require exactly one feedback ID");
        }
        auto& address = candidate.addresses[i];
        address.bus = ParseUnsignedInteger(motor["bus"], 3);
        address.command_id = ParseUnsignedInteger(motor["command_id"], 0x7ff);
        address.feedback_id = ParseUnsignedInteger(ids[0], 0x7ff);
        address.command_slot =
            candidate.protocols[i] == MotorProtocol::kDji
                ? ParseUnsignedInteger(motor["command_slot"], 3)
                : 0;
        candidate.freshness_us[i] = ParseFreshnessDeadline(
            motor["freshness_deadline_us"], UINT32_MAX - 1ULL);
        if (i < 2 && candidate.protocols[i] == MotorProtocol::kDji) {
            candidate.wheel_command_sign[i] =
                FiniteNumber(motor["command_sign"]);
        }
    }
    ValidateMotorConfig(candidate);
    return candidate;
}
void ValidateMotorAdapterConfig(const MotorConfig& config) {
    ValidateMotorConfig(config);
    std::string owner;
    for (size_t i = 0; i < config.present.size(); ++i) {
        if (!config.present[i]) {
            continue;
        }
        if (owner.empty()) {
            owner = config.adapters[i];
        }
        if (owner != config.adapters[i]) {
            throw std::invalid_argument(
                "A motor encoder/decoder requires a single adapter "
                "configuration");
        }
    }
}
MotorConfig MotorConfigForAdapter(const MotorConfig& config,
                                  const std::string& adapter) {
    auto selected = config;
    for (size_t i = 0; i < selected.present.size(); ++i) {
        selected.present[i] =
            config.present[i] && config.adapters[i] == adapter;
    }
    ValidateMotorAdapterConfig(selected);
    return selected;
}
}  // namespace dust_control::devices
