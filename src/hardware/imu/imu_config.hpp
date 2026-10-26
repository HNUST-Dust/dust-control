/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <cstdint>
#include <string>

namespace dust_control::devices {
struct ImuConfig {
    uint64_t freshness_us = 3000;
};
ImuConfig LoadImuConfig(const std::string& file);
}  // namespace dust_control::devices
