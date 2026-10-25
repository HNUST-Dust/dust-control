/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "bridge_stream.hpp"
#include "bridge_timing.hpp"

namespace dust_control::benchmarks::bridge_wire {
constexpr uint32_t kControlCommand = 0x32444d43;  // CMD2：原始反馈时间戳。
constexpr uint32_t kControlQuery = 0x31514c43;   // CLQ1
constexpr uint32_t kControlReply = 0x31524c43;   // CLR1
constexpr uint32_t kHardwareQuery = 0x31515748;  // HWQ1
constexpr uint32_t kHardwareReply =
    0x31525748;  // HWR1；标志位于 8，触发时间位于 16/20/24。
constexpr uint32_t kControlUnavailable = 1U << 31;
// 每条记录使用 MCU 时钟的绝对时间，显式标记数据是否存在，
// 因此零时间戳和 32 位环绕均有效；TX 完成时间对应整个批次。
enum class ControlPoint : unsigned { Origin, Driver, BatchDone, NextFeedback };
inline uint32_t ControlTime(const Packet &p, unsigned item,
                            ControlPoint point) {
    return ReadU32(p.data() + FrameOffset(item) + 4 * unsigned(point));
}

inline void SetControlTime(Packet &p, unsigned item, ControlPoint point,
                           uint32_t time) {
    WriteU32(p.data() + FrameOffset(item) + 4 * unsigned(point), time);
}

inline uint32_t ControlBit(unsigned item, ControlPoint point) {
    return 1U << (8 * unsigned(point) + item);
}

inline bool HasControlTime(const Packet &p, unsigned item, ControlPoint point) {
    return (ReadU32(p.data() + 8) & ControlBit(item, point)) != 0;
}

inline Packet ControlQuery(uint32_t sequence) {
    Packet p{};
    WriteU32(p.data(), kControlQuery);
    WriteU32(p.data() + 4, sequence);
    return p;
}

inline bool ValidControlReply(const Packet &p, uint32_t sequence) {
    if (ReadU32(p.data()) != kControlReply ||
        ReadU32(p.data() + 4) != sequence) {
        return false;
    }

    const uint32_t flags = ReadU32(p.data() + 8);
    return flags == kControlUnavailable || !(flags & ~0x07070707U);
}

inline Packet HardwareQuery(uint32_t sequence) {
    Packet p = ControlQuery(sequence);
    WriteU32(p.data(), kHardwareQuery);
    return p;
}

inline bool ValidHardwareReply(const Packet &p, uint32_t sequence) {
    const auto flags = ReadU32(p.data() + 8);
    return ReadU32(p.data()) == kHardwareReply &&
           ReadU32(p.data() + 4) == sequence &&
           (flags == kControlUnavailable || !(flags & ~7U));
}

// 有界诊断记录器，由调用者屏蔽 IRQ 后访问。
// 发布每组命令前在调度锁内暂存元数据，驱动标记与在途标记分开保存，
// 防止新命令覆盖旧传输的身份；测量期间不增加 USB 报文。
struct ControlLatencyRecorder {
    struct Tag {
        int index = -1;
        unsigned item = 0;
    };

    Packet samples[kTimingCapacity]{};
    uint32_t hardware[kTimingCapacity][4]{};  // 存在掩码后接三个触发时间。
    Tag staged[6]{}, flight[6]{}, awaiting_feedback[6]{};

    void Stage(const Packet &command, unsigned group, uint32_t receive_us) {
        const uint32_t seq = ReadU32(command.data() + 4);
        const unsigned index = TimingIndex(seq);
        const bool sampled = SampleTiming(seq) && index < kTimingCapacity;
        if (sampled) {
            auto &p = samples[index];
            for (auto &value : hardware[index]) {
                value = 0;
            }
            p = {};
            WriteU32(p.data(), kControlReply);
            WriteU32(p.data() + 4, seq);
            WriteU32(p.data() + 12, receive_us);
            for (unsigned i = 0; i < 3; ++i) {
                if (ReadU32(command.data() + 12) & (1U << i)) {
                    Mark({int(index), i}, ControlPoint::Origin,
                         ReadU32(command.data() + FrameOffset(i) + 12));
                }
            }
        }
        for (unsigned i = 0; i < 3; ++i) {
            staged[group * 3 + i] = {sampled ? int(index) : -1, i};
        }
    }

    void Mark(Tag tag, ControlPoint point, uint32_t us) {
        if (tag.index < 0) {
            return;
        }
        auto &p = samples[tag.index];
        SetControlTime(p, tag.item, point, us);
        WriteU32(p.data() + 8,
                 ReadU32(p.data() + 8) | ControlBit(tag.item, point));
    }

    void Driver(unsigned motor, uint32_t us) {
        flight[motor] = staged[motor];
        staged[motor] = {};
        Mark(flight[motor], ControlPoint::Driver, us);
    }

    void HardwareStart(unsigned motor, uint32_t us) {
        const auto tag = flight[motor];
        if (tag.index < 0) {
            return;
        }
        hardware[tag.index][0] |= 1U << tag.item;
        hardware[tag.index][1 + tag.item] = us;
    }

    Packet QueryHardware(uint32_t sequence) const {
        Packet p = HardwareQuery(sequence);
        WriteU32(p.data(), kHardwareReply);
        if (ReadU32(Query(sequence).data() + 8) == kControlUnavailable) {
            WriteU32(p.data() + 8, kControlUnavailable);
            return p;
        }
        const auto &record = hardware[TimingIndex(sequence)];
        WriteU32(p.data() + 8, record[0]);
        for (unsigned i = 0; i < 3; ++i) {
            WriteU32(p.data() + 16 + 4 * i, record[1 + i]);
        }
        return p;
    }

    void BatchDone(unsigned motor, uint32_t us) {
        Mark(flight[motor], ControlPoint::BatchDone, us);
        if (flight[motor].index >= 0) {
            awaiting_feedback[motor] = flight[motor];
        }
        flight[motor] = {};
    }

    void Receive(unsigned motor, uint32_t us) {
        auto &tag = awaiting_feedback[motor];
        if (tag.index < 0) {
            return;
        }
        const auto &p = samples[tag.index];
        // 回调携带的时间戳早于完成时间时，拒绝该延迟回调。
        if (int32_t(us - ControlTime(p, tag.item, ControlPoint::BatchDone)) <
            0) {
            return;
        }
        Mark(tag, ControlPoint::NextFeedback, us);
        tag = {};
    }

    Packet Query(uint32_t sequence) const {
        const unsigned index = TimingIndex(sequence);
        if (SampleTiming(sequence) && index < kTimingCapacity &&
            ReadU32(samples[index].data()) == kControlReply &&
            ReadU32(samples[index].data() + 4) == sequence) {
            return samples[index];
        }
        Packet p = ControlQuery(sequence);
        WriteU32(p.data(), kControlReply);
        WriteU32(p.data() + 8, kControlUnavailable);
        return p;
    }
};
}  // namespace dust_control::benchmarks::bridge_wire
