/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <span>

#include "concurrency/published_snapshot.hpp"
#include "hardware/byte_stream.hpp"
#include "hardware/remote/remote_input_state.hpp"

namespace dust::devices {
struct RemoteSample {
    msg::RemoteInputState remote{};
    bool disable_pose = false;
};

// Read 可跨线程调用；其余操作由接收线程独占，输入字节仅在调用期间借用。
class SbusDecoder {
   public:
    void Reset();
    void Discontinuity();
    void Consume(std::span<const uint8_t> bytes, uint64_t receive_us);
    bool PollTimeout(uint64_t now_us);
    bool timeout_pending() const {
        return lost_active_ && !lost_disabled_;
    }
    RemoteSample Read() const {
        return published_.Read();
    }

   private:
    void DecodeFrame(uint64_t receive_us);
    ByteStream<512> stream_;
    RemoteSample latest_;
    concurrency::PublishedSnapshot<RemoteSample> published_;
    uint32_t sequence_ = 0;
    bool lost_active_ = false;
    bool lost_disabled_ = false;
    uint64_t lost_start_us_ = 0;
};
}  // namespace dust::devices
