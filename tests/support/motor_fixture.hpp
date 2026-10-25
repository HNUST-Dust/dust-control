/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "hardware/motors/motor_config.hpp"

namespace dust::test {
inline constexpr std::array<devices::MotorAddress, 6> kMotorAddresses{{
    {3, 0x200, 0x201, 0},
    {2, 0x200, 0x201, 0},
    {3, 0x000, 0x010, 0},
    {3, 0x003, 0x013, 0},
    {2, 0x001, 0x011, 0},
    {2, 0x002, 0x012, 0},
}};
// 离线契约的明确接线，不参与生产构建或 YAML 加载。
inline devices::MotorConfig MotorFixture() {
    devices::MotorConfig config;
    config.adapters.fill("base");
    config.present.fill(true);
    config.addresses = kMotorAddresses;
    return config;
}
}  // namespace dust::test
