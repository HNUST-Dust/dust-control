/* SPDX-License-Identifier: Apache-2.0 */
#include "hi91_decoder.hpp"

#include <algorithm>
#include <cmath>

#include "hardware/imu/hi91_protocol.h"

namespace dust::devices {
namespace {
constexpr double kMaximumValidPitchDeg = 100.0;
}  // namespace

namespace {
uint16_t Read16(const uint8_t* bytes) {
    return uint16_t(bytes[0]) | (uint16_t(bytes[1]) << 8);
}
}  // namespace

void Hi91Decoder::Reset() {
    stream_ = {};
    latest_ = {};
    time_guard_.Reset();
    sequence_ = 0;
    published_.Publish(latest_);
}

void Hi91Decoder::Discontinuity() {
    stream_.size = 0;
    time_guard_.Reject();
    latest_.valid = false;
    published_.Publish(latest_);
}

void Hi91Decoder::Consume(std::span<const uint8_t> bytes, uint64_t sample_us) {
    for (const auto byte : bytes) {
        stream_.Append(byte);
        DecodeFrames(sample_us);
    }
    published_.Publish(latest_);
}

void Hi91Decoder::DecodeFrames(uint64_t sample_us) {
    for (;;) {
        if (stream_.size < 2) {
            return;
        }
        if (stream_.bytes[0] != 0x5a || stream_.bytes[1] != 0xa5) {
            time_guard_.Reject();
            stream_.DiscardPrefix(1);
            continue;
        }
        if (stream_.size < 6) {
            return;
        }
        const size_t payload = Read16(stream_.bytes.data() + 2);
        if (payload > protocols::kHi91MaxPayloadLength ||
            payload < protocols::kHi91DataLength) {
            time_guard_.Reject();
            stream_.DiscardPrefix(1);
            continue;
        }
        const size_t size = payload + protocols::kHi91FrameHeaderSize;
        if (stream_.size < size) {
            return;
        }
        protocols::Hi91Sample decoded{};
        const int result = protocols::DecodeHi91Frame(
            stream_.bytes.data(), size, strict_crc_, &decoded);
        if (result) {
            time_guard_.Reject();
            stream_.DiscardPrefix(1);
            continue;
        }
        if (!std::isfinite(decoded.pitch_deg) ||
            std::abs(decoded.pitch_deg) > kMaximumValidPitchDeg ||
            !std::isfinite(decoded.roll_deg) ||
            !std::isfinite(decoded.yaw_deg)) {
            time_guard_.Reject();
            stream_.DiscardPrefix(1);
            continue;
        }
        bool finite = true;
        for (unsigned i = 0; i < 3; ++i) {
            finite = finite && std::isfinite(decoded.gyro_dps[i]) &&
                     std::isfinite(decoded.accel_g[i]);
        }
        if (!finite) {
            time_guard_.Reject();
            stream_.DiscardPrefix(size);
            continue;
        }
        // 宽松 CRC 模式下，恢复候选帧仍必须通过校验。
        const bool crc_valid =
            Read16(stream_.bytes.data() + 4) ==
            protocols::CalculateHi91FrameCrc(payload, stream_.bytes.data() + 6);
        if (!time_guard_.Accept(decoded.system_time_ms, sample_us,
                                config_.freshness_us, crc_valid)) {
            stream_.DiscardPrefix(size);
            continue;
        }
        latest_ = {};
        latest_.sequence = ++sequence_;
        latest_.precise_timestamp_us = sample_us;
        latest_.valid = true;
        latest_.pitch_deg = decoded.pitch_deg;
        latest_.roll_deg = decoded.roll_deg;
        latest_.yaw_deg = decoded.yaw_deg;
        constexpr float radians = 0.01745329251994329577F;
        // 保留 HI91 安装映射：陀螺仪 X 对应 pitch，Y 对应 roll。
        latest_.pitch_rate_rad_s = decoded.gyro_dps[0] * radians;
        latest_.roll_rate_rad_s = decoded.gyro_dps[1] * radians;
        latest_.yaw_rate_rad_s = decoded.gyro_dps[2] * radians;
        std::copy_n(decoded.accel_g, 3, latest_.accel_g);
        latest_.vertical_accel_g = decoded.accel_g[2];
        stream_.DiscardPrefix(size);
    }
}
}  // namespace dust::devices
