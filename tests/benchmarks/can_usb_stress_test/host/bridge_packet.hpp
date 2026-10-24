/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "echo_packet.hpp"

namespace bridge_wire {
using stress_host::Packet;
using stress_host::ReadU32;
using stress_host::WriteU32;
constexpr uint32_t kRequestMagic = 0x314e4143;  // CAN1
constexpr uint32_t kReplyMagic = 0x324e4143;    // CAN2
constexpr uint32_t kBusy = 1U << 8;
constexpr uint32_t kInvalid = 1U << 9;
constexpr uint32_t kStopped = 1U << 10;
constexpr uint32_t kCanError = 1U << 11;
constexpr uint32_t kMissingMask = 7;
constexpr unsigned kFrames = 3;
constexpr unsigned FrameOffset(unsigned index) {
    return 16 + 16 * index;
}

inline uint16_t ReadId(const unsigned char *p) {
    return p[2] | (uint16_t(p[3]) << 8);
}

inline void SetAddress(unsigned char *p, uint8_t bus, uint16_t id,
                       uint8_t dlc) {
    p[0] = bus;
    p[1] = dlc;
    p[2] = id;
    p[3] = id >> 8;
}

// 头部依次为 magic、主机序列号、前次 RTT/请求或状态/回复、MCU OUT 接收时间。
// 随后为三条记录：bus/u8、dlc/u8、CAN id/u16、原始数据[8]、
// OUT 后反馈时间/us/u32。回复等待三帧批次后的反馈齐全，否则标记缺失位。
}  // namespace bridge_wire

#include "bridge_layout.hpp"

namespace bridge_wire {
inline Packet EncodeRequest(uint32_t sequence, uint32_t previous_rtt_us,
                            unsigned group) {
    Packet packet{};
    WriteU32(packet.data(), kRequestMagic);
    WriteU32(packet.data() + 4, sequence);
    WriteU32(packet.data() + 8, previous_rtt_us);
    for (unsigned item = 0; item < 3; ++item) {
        const auto &motor = kLayout[group][item];
        auto *frame = packet.data() + FrameOffset(item);
        SetAddress(frame, motor.bus, motor.command_id, 8);
        for (unsigned byte = 0; byte < 8; ++byte) {
            frame[4 + byte] = motor.zero[byte];
        }
    }

    return packet;
}

inline bool ValidateReply(const Packet &packet, unsigned group) {
    if (ReadU32(packet.data()) != kReplyMagic) {
        return false;
    }

    const uint32_t status = ReadU32(packet.data() + 8);
    if (status & ~(kMissingMask | kBusy | kInvalid | kStopped | kCanError)) {
        return false;
    }

    if (status & ~kMissingMask) {
        return true;  // 请求被拒绝，无 CAN 记录。
    }

    for (unsigned item = 0; item < 3; ++item) {
        const auto *frame = packet.data() + FrameOffset(item);
        const auto &motor = kLayout[group][item];
        if (frame[0] != motor.bus || ReadId(frame) != motor.feedback_id) {
            return false;
        }
        if (status & (1U << item)) {
            if (frame[1] != 0) {
                return false;
            }
        } else if (frame[1] != 8 || ReadU32(frame + 12) > 100000) {
            return false;
        }
    }

    return true;
}
}  // namespace bridge_wire
