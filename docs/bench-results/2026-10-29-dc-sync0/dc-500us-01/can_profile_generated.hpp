/* SPDX-License-Identifier: Apache-2.0 */
// Generated from config/can_devices.yaml; do not edit.
#pragma once
#include <array>
#include "usb_protocol.hpp"
namespace dust::can_profile {
struct Bus { uint8_t channel; uint32_t bitrate; };
struct Address { uint8_t channel; uint32_t id; uint8_t flags; };
inline constexpr uint32_t kCommandTimeoutUs = 20000U;
inline constexpr bool kProtectOnBoot = true;
inline constexpr uint32_t kProtectionSignature = 0xf7ebfae4U;
inline constexpr std::array<Bus, 4> kBuses = {{
    {3, 1000000U},
    {0, 1000000U},
    {1, 1000000U},
    {2, 1000000U},
}};
inline constexpr std::array<Address, 1> kReceiveFilters = {{
    {3, 0x13U, 0},
}};
inline constexpr std::array<Address, 1> kTransmitAddresses = {{
    {3, 0x3U, 0},
}};
inline constexpr std::array<wire::CanFrame, 1> kProtectionFrames = {{
    {3, 8, 0, 0x3U, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd}},
}};
} // namespace dust::can_profile
