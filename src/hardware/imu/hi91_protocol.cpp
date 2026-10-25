/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file src/hardware/imu/hi91_protocol.cpp
 * @ingroup wbr_protocols
 * @brief 实现 HI91 IMU 串口协议的帧解析。
 * @details
 * 实现显式处理字节序、帧长度和量化范围，不依赖动态内存。所有协议错误通过返回值报告，解析器不会直接驱动执行器。
 */

#include <errno.h>
#include <hardware/imu/hi91_protocol.h>
#include <string.h>

#include <algorithm>
#include <cmath>

namespace dust_control::protocols {

namespace {

uint16_t ReadLe16(const uint8_t *data) {
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t ReadLe32(const uint8_t *data) {
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8U) |
           (static_cast<uint32_t>(data[2]) << 16U) |
           (static_cast<uint32_t>(data[3]) << 24U);
}

float ReadLeFloat(const uint8_t *data) {
    const uint32_t raw = ReadLe32(data);
    float value = 0.0F;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

void DecodeFloat3(const uint8_t *data, float value[3]) {
    value[0] = ReadLeFloat(&data[0]);
    value[1] = ReadLeFloat(&data[4]);
    value[2] = ReadLeFloat(&data[8]);
}

uint16_t Crc16Update(uint16_t crc, const uint8_t *data, size_t size) {
    for (size_t i = 0U; i < size; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
            } else {
                crc = static_cast<uint16_t>(crc << 1U);
            }
        }
    }

    return crc;
}

}  // namespace

uint16_t CalculateHi91FrameCrc(uint16_t payload_length,
                               const uint8_t *payload) {
    const uint8_t header[4] = {
        kHi91FrameSof0,
        kHi91FrameSof1,
        static_cast<uint8_t>(payload_length & 0xFFU),
        static_cast<uint8_t>((payload_length >> 8U) & 0xFFU),
    };
    /* HiPNUC manual: start from 0, then update over SOF+LEN and payload
     * separately. CRC bytes themselves are excluded. */
    uint16_t crc = 0U;
    crc = Crc16Update(crc, header, sizeof(header));
    return Crc16Update(crc, payload, payload_length);
}

int DecodeHi91Frame(const uint8_t *data, size_t len, bool strict_crc,
                    Hi91Sample *out) {
    if ((data == nullptr) || (out == nullptr)) {
        return -EINVAL;
    }

    if (len < kHi91FrameHeaderSize) {
        return -EINVAL;
    }

    if ((data[0] != kHi91FrameSof0) || (data[1] != kHi91FrameSof1)) {
        return -EBADMSG;
    }

    const uint16_t payload_length = ReadLe16(&data[2]);
    if ((payload_length == 0U) || (payload_length > kHi91MaxPayloadLength)) {
        return -EBADMSG;
    }

    if (len < kHi91FrameHeaderSize + payload_length) {
        return -EMSGSIZE;
    }

    const uint8_t *payload = &data[kHi91FrameHeaderSize];
    if (strict_crc && (CalculateHi91FrameCrc(payload_length, payload) !=
                       ReadLe16(&data[4]))) {
        return -EILSEQ;
    }

    if ((payload_length < kHi91DataLength) || (payload[0] != kHi91DataTag)) {
        return -EBADMSG;
    }

    out->main_status = ReadLe16(&payload[1]);
    out->temperature_c = static_cast<int8_t>(payload[3]);
    out->air_pressure = ReadLeFloat(&payload[4]);
    out->system_time_ms = ReadLe32(&payload[8]);
    DecodeFloat3(&payload[12], out->accel_g);
    DecodeFloat3(&payload[24], out->gyro_dps);
    DecodeFloat3(&payload[36], out->mag_ut);
    out->roll_deg = ReadLeFloat(&payload[48]);
    out->pitch_deg = ReadLeFloat(&payload[52]);
    out->yaw_deg = ReadLeFloat(&payload[56]);
    out->quat[0] = ReadLeFloat(&payload[60]);
    out->quat[1] = ReadLeFloat(&payload[64]);
    out->quat[2] = ReadLeFloat(&payload[68]);
    out->quat[3] = ReadLeFloat(&payload[72]);
    return 0;
}

uint16_t Hi91Crc16CcittFalse(const uint8_t *data, size_t size) {
    return Crc16Update(0U, data, size);
}

}  // namespace dust_control::protocols

namespace dust_control::devices {
namespace {
constexpr double kMaximumValidPitchDeg = 100.0;
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
        if (stream_.bytes[0] != dust_control::protocols::kHi91FrameSof0 ||
            stream_.bytes[1] != dust_control::protocols::kHi91FrameSof1) {
            time_guard_.Reject();
            stream_.DiscardPrefix(1);
            continue;
        }
        if (stream_.size < dust_control::protocols::kHi91FrameHeaderSize) {
            return;
        }
        const size_t payload =
            dust_control::protocols::ReadLe16(stream_.bytes.data() + 2);
        if (payload > dust_control::protocols::kHi91MaxPayloadLength ||
            payload < dust_control::protocols::kHi91DataLength) {
            time_guard_.Reject();
            stream_.DiscardPrefix(1);
            continue;
        }
        const size_t size =
            payload + dust_control::protocols::kHi91FrameHeaderSize;
        if (stream_.size < size) {
            return;
        }
        dust_control::protocols::Hi91Sample decoded{};
        const int result = dust_control::protocols::DecodeHi91Frame(
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
            dust_control::protocols::ReadLe16(stream_.bytes.data() + 4) ==
            dust_control::protocols::CalculateHi91FrameCrc(
                payload, stream_.bytes.data() +
                             dust_control::protocols::kHi91FrameHeaderSize);
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
}  // namespace dust_control::devices
