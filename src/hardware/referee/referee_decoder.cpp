/* SPDX-License-Identifier: Apache-2.0 */
#include "referee_decoder.hpp"

#include <algorithm>

#include "hardware/referee/referee_protocol.h"

namespace dust::devices {
static_assert(protocols::kRefereeMaxDataLength <=
              RefereeSample{}.payload.size());
static_assert(protocols::kRefereeMaxFrameSize <= 160);

void RefereeDecoder::Reset() {
    stream_ = {};
    latest_ = {};
    published_.Publish(latest_);
}

void RefereeDecoder::Discontinuity() {
    // 保留已有观测及其采样时间，丢弃尚未组装完整的帧。
    stream_.size = 0;
}

void RefereeDecoder::Consume(std::span<const uint8_t> bytes,
                             uint64_t receive_us) {
    const size_t count = std::min(bytes.size(), latest_.chunk.size());
    std::copy_n(bytes.begin(), count, latest_.chunk.begin());
    latest_.chunk_size = static_cast<uint8_t>(count);
    latest_.receive_us = receive_us;
    for (const auto byte : bytes) {
        stream_.Append(byte);
        DecodeFrames(receive_us);
    }
    published_.Publish(latest_);
}

void RefereeDecoder::DecodeFrames(uint64_t receive_us) {
    while (stream_.size) {
        if (stream_.bytes[0] != protocols::kRefereeSof) {
            stream_.DiscardPrefix(1);
            continue;
        }
        if (stream_.size < protocols::kRefereeFrameHeaderSize) {
            return;
        }
        const size_t payload_size =
            uint16_t(stream_.bytes[1]) | (uint16_t(stream_.bytes[2]) << 8);
        if (payload_size > protocols::kRefereeMaxDataLength) {
            stream_.DiscardPrefix(1);
            continue;
        }
        const size_t frame_size = protocols::kRefereeFrameHeaderSize +
                                  payload_size +
                                  protocols::kRefereeFrameTrailerSize;
        if (stream_.size < frame_size) {
            return;
        }
        protocols::RefereeFrame frame{};
        if (protocols::DecodeRefereeFrame(stream_.bytes.data(), frame_size,
                                          &frame)) {
            stream_.DiscardPrefix(1);
            continue;
        }
        latest_.payload = {};
        std::copy_n(frame.data, frame.data_len, latest_.payload.begin());
        latest_.payload_size = frame.data_len;
        latest_.command_id = frame.cmd_id;
        latest_.frame_us = receive_us;
        ++latest_.frame_sequence;
        stream_.DiscardPrefix(frame_size);
    }
}
}  // namespace dust::devices
