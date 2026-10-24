/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include "dm_protocol.h"

namespace platform::motor_model {
// 当前关节电机型号的 MIT 量程；编码与反馈解码共用。
inline constexpr protocols::DmMitRange kJointMitRange{
    -12.56637F, 12.56637F, -45.0F, 45.0F,  0.0F,
    500.0F,     0.0F,      5.0F,   -54.0F, 54.0F};
}  // namespace platform::motor_model
