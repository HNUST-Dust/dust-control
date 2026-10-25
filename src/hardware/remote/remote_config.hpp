/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <cstdint>
#include <string>

namespace dust_control::devices {
struct RemoteConfig {
    uint64_t freshness_us = 1000000;
};
RemoteConfig LoadRemoteConfig(const std::string& file);
}  // namespace dust_control::devices
