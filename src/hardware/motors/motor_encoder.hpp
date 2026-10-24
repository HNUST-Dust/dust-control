/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <optional>

#include "hardware/can_frame.hpp"
#include "hardware/motors/motor_targets.hpp"
#include "motor_config.hpp"

namespace dust::devices {
// 无状态编码；不持有 USB、回调或底盘控制器，不执行使能许可判断。
// 成功返回本周期 CAN 帧；无效目标返回 nullopt，不输出部分批次。
class MotorEncoder {
   public:
    explicit MotorEncoder(MotorConfig config = {}) : config_(config) {
        ValidateMotorConfig(config_);
    }
    std::optional<CanTransmitBatch> Encode(const MotorTargets& targets) const;

   private:
    const MotorConfig config_;
};
}  // namespace dust::devices
