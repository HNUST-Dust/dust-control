/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "concurrency/published_snapshot.hpp"
#include "hardware/motors/motor_state.hpp"
#include "motor_config.hpp"

namespace dust::devices {
// 输入字节仅在调用期间借用，返回值拥有自己的物理量。
MotorFeedback DecodeMotorFeedback(unsigned index, const uint8_t* bytes,
                                  size_t size, uint64_t sample_us);

// Read 可跨线程调用；CAN 路由与解码由接收线程独占，不依赖 USB 格式。
class MotorFeedbackDecoder {
   public:
    explicit MotorFeedbackDecoder(MotorConfig config = {}) : config_(config) {}
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
}  // namespace dust::devices
