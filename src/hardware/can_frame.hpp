/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstdint>

namespace dust_control::devices {
// 当前电机使用的标准 CAN 数据帧；拥有载荷，不借用编码器缓冲。
struct CanFrame {
    uint8_t bus = 0;
    uint16_t id = 0;
    std::array<uint8_t, 8> bytes{};
};
}  // namespace dust_control::devices
