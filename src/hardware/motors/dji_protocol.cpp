/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file src/hardware/motors/dji_protocol.cpp
 * @ingroup wbr_protocols
 * @brief 实现大疆电机 CAN 协议编解码。
 * @details
 * 实现显式处理字节序、帧长度和量化范围，不依赖动态内存。所有协议错误通过返回值报告，解析器不会直接驱动执行器。
 */

#include <errno.h>
#include <hardware/motors/dji_protocol.h>

#include <algorithm>
#include <cmath>

#include "motor_protocol.hpp"

namespace dust_control::protocols {

namespace {
uint16_t ReadBigEndian16(const uint8_t* p) {
    return (uint16_t(p[0]) << 8) | p[1];
}

int WriteBe16ToSlot(int16_t value, int slot, uint8_t out[8]) {
    if ((out == nullptr) || (slot < 0) || (slot > 3)) {
        return -EINVAL;
    }

    const uint16_t encoded = static_cast<uint16_t>(value);
    out[slot * 2] = static_cast<uint8_t>((encoded >> 8) & 0xFFU);
    out[slot * 2 + 1] = static_cast<uint8_t>(encoded & 0xFFU);
    return 0;
}

int EncodeCurrentFrameCommon(const int16_t current_cmd[4], uint8_t out[8]) {
    if ((current_cmd == nullptr) || (out == nullptr)) {
        return -EINVAL;
    }

    for (int i = 0; i < 4; ++i) {
        const int rc = WriteBe16ToSlot(current_cmd[i], i, out);
        if (rc != 0) {
            return rc;
        }
    }

    return 0;
}

}  // namespace

bool IsDjiStandardFeedbackId(uint16_t can_id) {
    return (can_id >= 0x201U) && (can_id <= 0x204U);
}

int DecodeDjiFeedback(const uint8_t* data, uint8_t dlc, DjiMotorFeedback* out) {
    if ((data == nullptr) || (out == nullptr) || (dlc < 7U)) {
        return -EINVAL;
    }

    out->encoder = ReadBigEndian16(&data[0]);
    out->omega = static_cast<int16_t>(ReadBigEndian16(&data[2]));
    out->current = static_cast<int16_t>(ReadBigEndian16(&data[4]));
    out->temperature = data[6];
    return 0;
}

int EncodeDjiCurrentFrame0x200(const int16_t current_cmd[4], uint8_t out[8]) {
    return EncodeCurrentFrameCommon(current_cmd, out);
}

int EncodeDjiCurrentFrame0x1ff(const int16_t current_cmd[4], uint8_t out[8]) {
    return EncodeCurrentFrameCommon(current_cmd, out);
}

int WriteDjiCurrentCommandToSlot(uint16_t motor_can_id, int16_t current_cmd,
                                 uint8_t frame_payload[8]) {
    int slot = -1;
    if ((motor_can_id >= 0x201U) && (motor_can_id <= 0x204U)) {
        slot = static_cast<int>(motor_can_id - 0x201U);
    } else {
        return -EINVAL;
    }

    return WriteBe16ToSlot(current_cmd, slot, frame_payload);
}

}  // namespace dust_control::protocols

namespace dust_control::protocols {
bool EncodeDjiMotorTarget(const dust_control::devices::MotorTarget& target,
                          uint8_t command_slot, double command_sign,
                          uint8_t out[8]) {
    if (target.mode != dust_control::devices::MotorMode::kTorque ||
        command_slot >= 4 || !out) {
        return false;
    }
    const double requested = command_sign * target.torque_nm;
    const double current =
        std::isfinite(requested)
            ? std::clamp(
                  requested *
                      dust_control::platform::motor_model::kDjiCurrentPerNm,
                  -static_cast<double>(dust_control::platform::motor_model::
                                           kDjiProtocolCurrentLimit),
                  static_cast<double>(dust_control::platform::motor_model::
                                          kDjiProtocolCurrentLimit))
            : 0.0;
    int16_t currents[4]{};
    currents[command_slot] = static_cast<int16_t>(current);
    return EncodeDjiCurrentFrame0x200(currents, out) == 0;
}
dust_control::devices::MotorFeedback DecodeDjiMotorState(const uint8_t* bytes,
                                                         size_t size,
                                                         uint64_t sample_us) {
    dust_control::devices::MotorFeedback sample{};
    sample.precise_timestamp_us = sample_us;
    if (size != 8 || !bytes) {
        return sample;
    }
    DjiMotorFeedback decoded{};
    sample.valid = DecodeDjiFeedback(bytes, size, &decoded) == 0;
    sample.speed_rpm = static_cast<double>(decoded.omega);
    sample.velocity_rad_s = sample.speed_rpm * 0.10471975511965977;
    return sample;
}
}  // namespace dust_control::protocols
