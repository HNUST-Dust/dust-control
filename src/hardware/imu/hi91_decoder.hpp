/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <span>

#include "concurrency/published_snapshot.hpp"
#include "hardware/byte_stream.hpp"
#include "hi91_time_guard.hpp"
#include "imu_config.hpp"
#include "imu_sample.hpp"

namespace dust::devices {
// Read 可跨线程调用；拼帧、CRC 和时钟恢复状态只由接收线程访问。
class Hi91Decoder {
   public:
    explicit Hi91Decoder(bool strict_crc = false, ImuConfig config = {})
        : config_(config), strict_crc_(strict_crc) {}
    void Reset();
    void Discontinuity();
    void Consume(std::span<const uint8_t> bytes, uint64_t sample_us);
    ImuSample Read() const {
        return published_.Read();
    }

   private:
    const ImuConfig config_;
    void DecodeFrames(uint64_t sample_us);
    ByteStream<512> stream_;
    ImuSample latest_;
    concurrency::PublishedSnapshot<ImuSample> published_;
    wbr::host::Hi91TimeGuard time_guard_;
    uint32_t sequence_ = 0;
    bool strict_crc_;
};
}  // namespace dust::devices
