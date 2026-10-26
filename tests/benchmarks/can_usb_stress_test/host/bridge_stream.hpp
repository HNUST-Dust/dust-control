/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "bridge_packet.hpp"

namespace dust_control::benchmarks::bridge_wire {
constexpr uint32_t kStreamCommand = 0x31444d43;  // CMD1
constexpr uint32_t kStreamAck =
    0x314b4341;  // ACK1：仅表示提交，不代表电机响应。
constexpr uint32_t kStreamFeedback = 0x314b4246;    // FBK1
constexpr uint32_t kStreamStatsQuery = 0x31515353;  // SSQ1
constexpr uint32_t kStreamStatsReply = 0x31525353;  // SSR1
inline Packet StreamCommand(uint32_t sequence, uint32_t last_ack_us,
                            unsigned group) {
    auto packet = EncodeRequest(sequence, last_ack_us, group);
    WriteU32(packet.data(), kStreamCommand);
    return packet;
}

inline int FeedbackMotor(const unsigned char *frame) {
    for (unsigned group = 0; group < 2; ++group) {
        for (unsigned item = 0; item < 3; ++item) {
            if (frame[0] == kLayout[group][item].bus &&
                ReadId(frame) == kLayout[group][item].feedback_id) {
                return group * 3 + item;
            }
        }
    }

    return -1;
}

inline bool ValidFeedback(const Packet &packet) {
    const auto count = ReadU32(packet.data() + 8);
    if (ReadU32(packet.data()) != kStreamFeedback || count < 1 || count > 3) {
        return false;
    }

    for (unsigned i = 0; i < count; ++i) {
        const auto *frame = packet.data() + FrameOffset(i);
        if (frame[1] != 8 || FeedbackMotor(frame) < 0 ||
            uint32_t(ReadU32(packet.data() + 12) - ReadU32(frame + 12)) >
                1000000) {
            return false;
        }
    }

    return true;
}

inline bool ValidAck(const Packet &packet) {
    const auto status = ReadU32(packet.data() + 8);
    return ReadU32(packet.data()) == kStreamAck &&
           !(status & ~(kInvalid | kStopped | kCanError));
}

inline Packet StreamStatsQuery(uint32_t sequence, unsigned group) {
    Packet packet{};
    WriteU32(packet.data(), kStreamStatsQuery);
    WriteU32(packet.data() + 4, sequence);
    WriteU32(packet.data() + 8, group);
    return packet;
}

constexpr unsigned kStreamStatsPages = 4;
inline bool ValidStreamStats(const Packet &packet, uint32_t sequence,
                             unsigned page) {
    if (page >= kStreamStatsPages ||
        ReadU32(packet.data()) != kStreamStatsReply ||
        ReadU32(packet.data() + 4) != sequence ||
        ReadU32(packet.data() + 8) != page ||
        ReadU32(packet.data() + 12) == 0) {
        return false;
    }

    return page < 2 || ReadU32(packet.data() + 40) == kLayout[page - 2][0].bus;
}

// 有界的最新反馈邮箱，由外部 IRQ 锁保护访问。
// 覆盖待处理的电机数据不阻塞命令转发，只保留最新状态。
struct FeedbackMailbox {
    struct Sample {
        uint8_t bus = 0;
        uint16_t id = 0;
        uint8_t data[8]{};
        uint32_t received_us = 0;
    };

    Sample latest[6]{};
    uint8_t dirty = 0;
    unsigned next = 0;
    void Put(unsigned motor, const Sample &sample) {
        latest[motor] = sample;
        dirty |= 1U << motor;
    }

    Packet Take(uint32_t packet_sequence) {
        Packet packet{};
        WriteU32(packet.data(), kStreamFeedback);
        WriteU32(packet.data() + 4, packet_sequence);
        unsigned count = 0;
        for (unsigned checked = 0; checked < 6 && count < 3; ++checked) {
            const unsigned motor = next;
            next = (next + 1) % 6;
            if (!(dirty & (1U << motor))) {
                continue;
            }
            const auto &sample = latest[motor];
            auto *frame = packet.data() + FrameOffset(count++);
            SetAddress(frame, sample.bus, sample.id, 8);
            for (unsigned i = 0; i < 8; ++i) {
                frame[4 + i] = sample.data[i];
            }
            WriteU32(frame + 12, sample.received_us);
            dirty &= ~(1U << motor);
        }
        WriteU32(packet.data() + 8, count);
        return packet;
    }
};
}  // namespace dust_control::benchmarks::bridge_wire
