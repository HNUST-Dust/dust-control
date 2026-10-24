/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <cstdint>

namespace platform::motor_model {
// 当前 DJI 电流模式的命令规格，不是控制策略力矩上限。
// 当前轮电机与减速机构的默认电流/力矩换算，运行配置可覆盖。
inline constexpr double kDefaultDjiCurrentPerNm = 3450.0;
inline constexpr int16_t kDjiProtocolCurrentLimit = 16384;
inline constexpr uint16_t kDjiCommandId = 0x200;
inline constexpr uint16_t DjiFeedbackId(uint8_t slot) {
    return 0x201 + slot;
}
}  // namespace platform::motor_model
