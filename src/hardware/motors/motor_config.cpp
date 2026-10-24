/* SPDX-License-Identifier: Apache-2.0 */
#include "motor_config.hpp"

#include "hardware/config_yaml.hpp"

namespace dust::devices {
MotorConfig LoadMotorConfig(const std::string& file) {
    using namespace dust::config_yaml;
    MotorConfig candidate;
    const std::array<std::string, 6> names = {"left_wheel",    "right_wheel",
                                              "left_joint_b",  "left_joint_d",
                                              "right_joint_b", "right_joint_d"};
    const auto root = YAML::LoadFile(file);
    auto keys = std::set<std::string>(names.begin(), names.end());
    keys.insert("schema_version");
    ValidateKeys(root, keys);
    if (ParseUnsignedInteger(root["schema_version"], 1) != 1) {
        throw std::runtime_error("Unsupported motors schema version");
    }

    std::set<std::pair<unsigned, unsigned>> feedback;
    std::set<std::pair<unsigned, unsigned>> commands;
    for (size_t i = 0; i < 6; ++i) {
        const auto motor = root[names[i]];
        auto expected = std::set<std::string>{
            "bus", "command_id", "feedback_id", "freshness_deadline_us"};
        if (i < 2) {
            expected.insert("command_slot");
        }
        ValidateKeys(motor, expected);
        auto& address = candidate.addresses[i];
        address.bus = ParseUnsignedInteger(motor["bus"], 3);
        if (address.bus != 2 && address.bus != 3) {
            throw std::runtime_error("Gateway supports only CAN2/CAN3");
        }
        address.command_id = ParseUnsignedInteger(motor["command_id"], 0x7ff);
        address.feedback_id = ParseUnsignedInteger(motor["feedback_id"], 0x7ff);
        address.command_slot =
            i < 2 ? ParseUnsignedInteger(motor["command_slot"], 3) : 0;
        // 保持现有 DJI 编码器与反馈槽位的对应关系。
        if (i < 2 && (address.command_id != 0x200 ||
                      address.feedback_id != 0x201 + address.command_slot)) {
            throw std::runtime_error("DJI command/feedback slot mismatch");
        }
        if (!feedback.insert({address.bus, address.feedback_id}).second ||
            !commands.insert({address.bus, address.command_id}).second) {
            throw std::runtime_error("Duplicate motor CAN address");
        }
        candidate.freshness_us[i] = ParseFreshnessDeadline(
            motor["freshness_deadline_us"], UINT32_MAX - 1ULL);
    }

    return candidate;
}
}  // namespace dust::devices
