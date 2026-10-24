/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <hardware/usb/protocol/usb_protocol.hpp>

namespace wbr::bridge {
using namespace dust::wire;
constexpr uint32_t kCommandTimeoutUs = 20000;  // 应用选择的 MCU 租约。
}  // namespace wbr::bridge
