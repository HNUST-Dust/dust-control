/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstdint>
#include <span>

#include "concurrency/published_snapshot.hpp"
#include "hardware/byte_stream.hpp"

namespace dust::devices {
struct RefereeSample {
    std::array<uint8_t, 40> chunk{};
    uint8_t chunk_size = 0;
    uint64_t receive_us = 0;
    // 沿用现有裁判协议解析器：只校验帧边界，尚不校验 CRC8/CRC16。
    std::array<uint8_t, 151> payload{};
    uint16_t payload_size = 0;
    uint16_t command_id = 0;
    uint64_t frame_us = 0;
    uint32_t frame_sequence = 0;
};

// Read 可跨线程调用；其余操作仅由接收线程调用。输出不参与控制许可。
class RefereeDecoder {
   public:
    void Reset();
    void Discontinuity();
    void Consume(std::span<const uint8_t> bytes, uint64_t receive_us);
    RefereeSample Read() const {
        return published_.Read();
    }

   private:
    void DecodeFrames(uint64_t receive_us);
    ByteStream<160> stream_;
    RefereeSample latest_;
    concurrency::PublishedSnapshot<RefereeSample> published_;
};
}  // namespace dust::devices
