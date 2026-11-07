/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "concurrency/published_snapshot.hpp"
#include "hardware/can_frame.hpp"
#include "motor_config.hpp"
#include "motor_types.hpp"

namespace dust_control::devices {
// 输入字节仅在调用期间借用，返回值拥有自己的物理量。
MotorFeedback DecodeMotorFeedback(unsigned index, const uint8_t* bytes,
                                  size_t size, uint64_t sample_us,
                                  const MotorConfig& config);

// Read 可跨线程调用；CAN 路由与解码由接收线程独占，不依赖传输格式。
class MotorFeedbackDecoder {
   public:
    explicit MotorFeedbackDecoder(MotorConfig config) : config_(config) {
        ValidateMotorAdapterConfig(config_);
    }
    void Reset() {
        latest_ = {};
        published_.Publish(latest_);
    }
    void Consume(uint8_t bus, uint16_t id, std::span<const uint8_t> bytes,
                 uint64_t sample_us, bool timestamp_valid);
    std::array<MotorFeedback, 6> Read() const {
        return published_.Read();
    }

   private:
    const MotorConfig config_;
    std::array<MotorFeedback, 6> latest_{};
    concurrency::PublishedSnapshot<std::array<MotorFeedback, 6>> published_;
};

// 单块 Adapter 的电机编码结果；批次容量属于电机编码接口，不限制通用 CAN 帧。
struct CanTransmitBatch {
    std::array<CanFrame, 6> frames{};
    size_t count = 0;
};

// 无状态编码；不持有传输资源、回调或底盘控制器，不执行使能许可判断。
// 成功返回本周期 CAN 帧；无效目标返回 nullopt，不输出部分批次。
class MotorEncoder {
   public:
    explicit MotorEncoder(MotorConfig config) : config_(config) {
        ValidateMotorAdapterConfig(config_);
    }
    std::optional<CanTransmitBatch> Encode(const MotorTargets& targets) const;

   private:
    const MotorConfig config_;
};

}  // namespace dust_control::devices
