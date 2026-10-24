/* SPDX-License-Identifier: Apache-2.0 */
#include "sbus_decoder.hpp"

#include <algorithm>

#include "hardware/remote/wfly_sbus_protocol.h"

namespace dust::devices {
namespace {
// 当前 WFLY 遥控器的固定标定，开关阈值独立于摇杆归一化。
constexpr float kSbusMid = 1024.0F;
constexpr float kSbusScale = 670.0F;
constexpr uint16_t kSwitchLowMidThreshold = 689;
constexpr uint16_t kSwitchMidHighThreshold = 1359;
constexpr uint64_t kFrameLostTimeoutUs = 1000000;
}  // namespace

void SbusDecoder::Reset() {
    stream_ = {};
    latest_ = {};
    sequence_ = 0;
    lost_active_ = lost_disabled_ = false;
    lost_start_us_ = 0;
    published_.Publish(latest_);
}

void SbusDecoder::Discontinuity() {
    stream_.size = 0;
    latest_ = {};
    latest_.remote.sequence = ++sequence_;
    published_.Publish(latest_);
}

void SbusDecoder::Consume(std::span<const uint8_t> bytes, uint64_t receive_us) {
    for (const auto byte : bytes) {
        stream_.Append(byte);
        DecodeFrame(receive_us);
    }
    published_.Publish(latest_);
}

bool SbusDecoder::PollTimeout(uint64_t now_us) {
    if (!timeout_pending() || now_us < lost_start_us_ ||
        now_us - lost_start_us_ < kFrameLostTimeoutUs) {
        return false;
    }
    latest_ = {};
    latest_.remote.source = msg::kRemoteInputWfly;
    latest_.remote.sequence = ++sequence_;
    lost_disabled_ = true;
    published_.Publish(latest_);
    return true;
}

void SbusDecoder::DecodeFrame(uint64_t receive_us) {
    while (stream_.size && stream_.bytes[0] != protocols::kWflySbusStartByte) {
        stream_.DiscardPrefix(1);
    }
    if (stream_.size < protocols::kWflySbusFrameLength) {
        return;
    }
    protocols::WflySbusFrame decoded{};
    if (!protocols::DecodeWflySbusFrame(stream_.bytes.data(), 25, &decoded)) {
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
        stream_.DiscardPrefix(25);
        return;
    }
    lost_active_ = lost_disabled_ = false;
    msg::RemoteInputState remote{};
    remote.source = msg::kRemoteInputWfly;
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
    stream_.DiscardPrefix(25);
}
}  // namespace dust::devices
