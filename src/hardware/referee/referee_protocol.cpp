/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file src/hardware/referee/referee_protocol.cpp
 * @ingroup wbr_protocols
 * @brief 实现机器人裁判系统协议的帧解析。
 * @details
 * 实现显式处理字节序、帧长度和量化范围，不依赖动态内存。所有协议错误通过返回值报告，解析器不会直接驱动执行器。
 */

#include <errno.h>
#include <hardware/referee/referee_protocol.h>
#include <string.h>

#include <algorithm>

namespace dust_control::protocols {

namespace {

uint16_t ReadLe16(const uint8_t *data) {
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8U);
}

float ReadLeFloat(const uint8_t *data) {
    const uint32_t raw = static_cast<uint32_t>(data[0]) |
                         (static_cast<uint32_t>(data[1]) << 8U) |
                         (static_cast<uint32_t>(data[2]) << 16U) |
                         (static_cast<uint32_t>(data[3]) << 24U);
    float value = 0.0F;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

}  // namespace

int DecodeRefereeFrame(const uint8_t *frame, size_t frame_len,
                       RefereeFrame *out) {
    if ((frame == nullptr) || (out == nullptr) ||
        (frame_len < kRefereeFrameHeaderSize)) {
        return -EINVAL;
    }

    if (frame[0] != kRefereeSof) {
        return -EBADMSG;
    }

    const uint16_t data_len = ReadLe16(&frame[1]);
    if (data_len > kRefereeMaxDataLength) {
        return -EBADMSG;
    }

    const size_t full_len =
        kRefereeFrameHeaderSize + data_len + kRefereeFrameTrailerSize;
    if (frame_len < full_len) {
        return -EMSGSIZE;
    }

    out->cmd_id = ReadLe16(&frame[5]);
    out->data = &frame[kRefereeFrameHeaderSize];
    out->data_len = data_len;
    return 0;
}

int DecodeRefereeGameStatus(const uint8_t *data, size_t len,
                            RefereeGameStatus *out) {
    if ((data == nullptr) || (out == nullptr) || (len < 3U)) {
        return -EINVAL;
    }

    out->game_type = data[0] & 0x0fU;
    out->game_progress = static_cast<uint8_t>((data[0] >> 4) & 0x0fU);
    out->stage_remain_time = ReadLe16(&data[1]);
    return 0;
}

int DecodeRefereeRobotStatus(const uint8_t *data, size_t len,
                             RefereeRobotStatus *out) {
    if ((data == nullptr) || (out == nullptr) || (len < 13U)) {
        return -EINVAL;
    }

    out->current_hp = ReadLe16(&data[2]);
    out->max_hp = ReadLe16(&data[4]);
    out->chassis_power_limit = ReadLe16(&data[10]);
    out->gimbal_power_on = ((data[12] & 0x01U) != 0U) ? 1U : 0U;
    return 0;
}

int DecodeRefereeShootData(const uint8_t *data, size_t len,
                           RefereeShootData *out) {
    if ((data == nullptr) || (out == nullptr) || (len < 7U)) {
        return -EINVAL;
    }

    out->bullet_type = data[0];
    out->launching_frequency = data[2];
    out->initial_speed = ReadLeFloat(&data[3]);
    return 0;
}

}  // namespace dust_control::protocols

namespace dust_control::devices {
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
        if (stream_.bytes[0] != dust_control::protocols::kRefereeSof) {
            stream_.DiscardPrefix(1);
            continue;
        }
        if (stream_.size < dust_control::protocols::kRefereeFrameHeaderSize) {
            return;
        }
        const size_t payload_size =
            dust_control::protocols::ReadLe16(stream_.bytes.data() + 1);
        if (payload_size > dust_control::protocols::kRefereeMaxDataLength) {
            stream_.DiscardPrefix(1);
            continue;
        }
        const size_t frame_size =
            dust_control::protocols::kRefereeFrameHeaderSize + payload_size +
            dust_control::protocols::kRefereeFrameTrailerSize;
        if (stream_.size < frame_size) {
            return;
        }
        dust_control::protocols::RefereeFrame frame{};
        if (dust_control::protocols::DecodeRefereeFrame(stream_.bytes.data(),
                                                        frame_size, &frame)) {
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
}  // namespace dust_control::devices
