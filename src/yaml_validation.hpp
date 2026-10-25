
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>

namespace dust::yaml_validation {
inline void ValidateKeys(const YAML::Node &node,
                         const std::set<std::string> &allowed,
                         const std::set<std::string> &optional = {}) {
    if (!node.IsMap()) {
        throw std::runtime_error("Configuration must be a mapping");
    }

    std::set<std::string> seen;
    for (const auto &entry : node) {
        const auto key = entry.first.as<std::string>();
        if ((!allowed.count(key) && !optional.count(key)) ||
            !seen.insert(key).second) {
            throw std::runtime_error(
                "Unknown or duplicate configuration key: " + key);
        }
    }

    for (const auto &key : allowed) {
        if (!seen.contains(key)) {
            throw std::runtime_error("Missing configuration key: " + key);
        }
    }
}

inline uint64_t ParseUnsignedInteger(const YAML::Node &node, uint64_t maximum) {
    const auto text = node.as<std::string>();
    if (text.empty() || text[0] == '-' || text[0] == '+') {
        throw std::runtime_error("Expected unsigned integer");
    }

    size_t used = 0;
    const auto value = std::stoull(text, &used, 0);
    if (used != text.size() || value > maximum) {
        throw std::runtime_error("Configuration integer out of range");
    }

    return value;
}

inline uint64_t ParseFreshnessDeadline(const YAML::Node &node,
                                       uint64_t maximum) {
    const auto value = ParseUnsignedInteger(node, maximum);
    if (!value) {
        throw std::runtime_error("freshness_deadline_us must be positive");
    }

    return value;
}

inline YAML::Node LoadSensorFile(const std::string &file) {
    const auto root = YAML::LoadFile(file);
    ValidateKeys(root, {"schema_version", "hi91", "remote"});
    if (ParseUnsignedInteger(root["schema_version"], 1) != 1) {
        throw std::runtime_error("Unsupported sensor schema version");
    }
    return root;
}
}  // namespace dust::yaml_validation
