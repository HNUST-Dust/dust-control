/* SPDX-License-Identifier: Apache-2.0 */

/**
 * @file src/hardware/remote/wfly_sbus_protocol.cpp
 * @ingroup wbr_protocols
 * @brief 实现天地飞 SBUS 遥控协议解码。
 * @details
 * 实现显式处理字节序、帧长度和量化范围，不依赖动态内存。所有协议错误通过返回值报告，解析器不会直接驱动执行器。
 */

#include "wfly_sbus_protocol.h"

#include <algorithm>

namespace dust_control::protocols {

namespace {

constexpr uint8_t kChannel17Mask = 0x01;
constexpr uint8_t kChannel18Mask = 0x02;
constexpr uint8_t kFrameLostMask = 0x04;
constexpr uint8_t kFailsafeMask = 0x08;
constexpr uint16_t kChannelMask = 0x07ff;

uint16_t U16(uint8_t value) {
    return static_cast<uint16_t>(value);
}

}  // namespace

bool DecodeWflySbusFrame(const uint8_t *data, size_t len, WflySbusFrame *out) {
    if ((data == nullptr) || (out == nullptr) || (len < kWflySbusFrameLength)) {
        return false;
    }

    if ((data[0] != kWflySbusStartByte) || (data[24] != kWflySbusEndByte)) {
        return false;
    }

    const uint8_t *payload = &data[1];

    out->msg[0] = (U16(payload[0]) | (U16(payload[1]) << 8)) & kChannelMask;
    out->msg[1] =
        ((U16(payload[1]) >> 3) | (U16(payload[2]) << 5)) & kChannelMask;
    out->msg[2] = ((U16(payload[2]) >> 6) | (U16(payload[3]) << 2) |
                   (U16(payload[4]) << 10)) &
                  kChannelMask;
    out->msg[3] =
        ((U16(payload[4]) >> 1) | (U16(payload[5]) << 7)) & kChannelMask;
    out->msg[4] =
        ((U16(payload[5]) >> 4) | (U16(payload[6]) << 4)) & kChannelMask;
    out->msg[5] = ((U16(payload[6]) >> 7) | (U16(payload[7]) << 1) |
                   (U16(payload[8]) << 9)) &
                  kChannelMask;
    out->msg[6] =
        ((U16(payload[8]) >> 2) | (U16(payload[9]) << 6)) & kChannelMask;
    out->msg[7] =
        ((U16(payload[9]) >> 5) | (U16(payload[10]) << 3)) & kChannelMask;

    out->msg[8] = (U16(payload[11]) | (U16(payload[12]) << 8)) & kChannelMask;
    out->msg[9] =
        ((U16(payload[12]) >> 3) | (U16(payload[13]) << 5)) & kChannelMask;
    out->msg[10] = ((U16(payload[13]) >> 6) | (U16(payload[14]) << 2) |
                    (U16(payload[15]) << 10)) &
                   kChannelMask;
    out->msg[11] =
        ((U16(payload[15]) >> 1) | (U16(payload[16]) << 7)) & kChannelMask;
    out->msg[12] =
        ((U16(payload[16]) >> 4) | (U16(payload[17]) << 4)) & kChannelMask;
    out->msg[13] = ((U16(payload[17]) >> 7) | (U16(payload[18]) << 1) |
                    (U16(payload[19]) << 9)) &
                   kChannelMask;
    out->msg[14] =
        ((U16(payload[19]) >> 2) | (U16(payload[20]) << 6)) & kChannelMask;
    out->msg[15] =
        ((U16(payload[20]) >> 5) | (U16(payload[21]) << 3)) & kChannelMask;

    const uint8_t flags = data[23];
    out->channel17 = (flags & kChannel17Mask) != 0U;
    out->channel18 = (flags & kChannel18Mask) != 0U;
    out->frame_lost = (flags & kFrameLostMask) != 0U;
    out->failsafe = (flags & kFailsafeMask) != 0U;
    return true;
}

}  // namespace dust_control::protocols

namespace dust_control::devices {
namespace {
// 当前 WFLY 遥控器的固定标定，开关阈值独立于摇杆归一化。
constexpr float kSbusMid = 1024.0F;
constexpr float kSbusScale = 670.0F;
constexpr uint16_t kSwitchLowMidThreshold = 689;
constexpr uint16_t kSwitchMidHighThreshold = 1359;
constexpr uint64_t kFrameLostTimeoutUs = 1000000;
}  // namespace

void WflySbusDecoder::Reset() {
    stream_ = {};
    latest_ = {};
    sequence_ = 0;
    lost_active_ = lost_disabled_ = false;
    lost_start_us_ = 0;
    published_.Publish(latest_);
}

void WflySbusDecoder::Discontinuity() {
    stream_.size = 0;
    latest_ = {};
    latest_.remote.sequence = ++sequence_;
    published_.Publish(latest_);
}

void WflySbusDecoder::Consume(std::span<const uint8_t> bytes,
                              uint64_t receive_us) {
    for (const auto byte : bytes) {
        stream_.Append(byte);
        DecodeFrame(receive_us);
    }
    published_.Publish(latest_);
}

bool WflySbusDecoder::PollTimeout(uint64_t now_us) {
    if (!timeout_pending() || now_us < lost_start_us_ ||
        now_us - lost_start_us_ < kFrameLostTimeoutUs) {
        return false;
    }
    latest_ = {};
    latest_.remote.source = dust_control::input::kRemoteInputWfly;
    latest_.remote.sequence = ++sequence_;
    lost_disabled_ = true;
    published_.Publish(latest_);
    return true;
}

void WflySbusDecoder::DecodeFrame(uint64_t receive_us) {
    while (stream_.size &&
           stream_.bytes[0] != dust_control::protocols::kWflySbusStartByte) {
        stream_.DiscardPrefix(1);
    }
    if (stream_.size < dust_control::protocols::kWflySbusFrameLength) {
        return;
    }
    dust_control::protocols::WflySbusFrame decoded{};
    if (!dust_control::protocols::DecodeWflySbusFrame(
            stream_.bytes.data(), dust_control::protocols::kWflySbusFrameLength,
            &decoded)) {
        stream_.DiscardPrefix(1);
        return;
    }
    // 保留处理优先级：先丢弃 frame_lost，再处理 failsafe。
    if (decoded.frame_lost) {
        latest_.disable_pose = false;
        if (!lost_active_) {
            lost_active_ = true;
            lost_disabled_ = false;
            lost_start_us_ = receive_us;
        }
        PollTimeout(receive_us);
        stream_.DiscardPrefix(dust_control::protocols::kWflySbusFrameLength);
        return;
    }
    lost_active_ = lost_disabled_ = false;
    dust_control::input::RemoteInputState remote{};
    remote.source = dust_control::input::kRemoteInputWfly;
    auto normalized = [](uint16_t value) {
        return std::clamp((float(value) - kSbusMid) / kSbusScale, -1.0F, 1.0F);
    };
    auto position = [](uint16_t value) {
        return value < kSwitchLowMidThreshold    ? 0
               : value > kSwitchMidHighThreshold ? 2
                                                 : 1;
    };
    const auto ch4 = position(decoded.msg[4]);
    const auto ch5 = position(decoded.msg[5]);
    latest_.disable_pose = !decoded.failsafe && ch4 == 0 && ch5 == 0;
    if (!decoded.failsafe) {
        remote.robot_enable = (ch4 != 0 || ch5 != 0);
        remote.run = remote.robot_enable;
        if (remote.robot_enable) {
            remote.chassis_x = normalized(decoded.msg[2]);
            remote.chassis_rotate = normalized(decoded.msg[3]);
            if (ch4 == 0) {
                remote.yaw_angle = normalized(decoded.msg[0]);
                remote.pitch_angle = normalized(decoded.msg[1]);
            }
            if (ch4 == 1) {
                remote.leg_length_delta = normalized(decoded.msg[1]);
                remote.leg_length = remote.leg_length_delta;
            }
            if (ch4 == 2) {
                remote.chassis_x = remote.chassis_rotate = 0;
                remote.climb_stairs = true;
            }
            remote.jump = ch5 == 2;
        }
    }
    remote.sequence = ++sequence_;
    latest_.remote = remote;
    stream_.DiscardPrefix(dust_control::protocols::kWflySbusFrameLength);
}
}  // namespace dust_control::devices
