/* SPDX-License-Identifier: Apache-2.0 */
#include "usb_transmitter.hpp"

#include <cassert>
#include <iostream>

#include "usb_link.hpp"

namespace dust::test {
// 使用虚拟单调时钟检查调度边界，不依赖主机实时调度或真实 USB。
class UsbTransmitFixture {
   public:
    using Tx = wbr::host::UsbTransmitter;
    using Producer = wbr::host::CanProducer;
    using Group = wbr::host::CanCommandGroup;
    using Frame = dust::devices::CanFrame;
    using Kind = Tx::WorkKind;
    wbr::host::UartReceiveQueue queue;
    wbr::host::UsbLink link{queue, {}};
    Tx tx{link};
    static constexpr uint64_t kStart = 1000000000;
    static constexpr uint64_t kTimeout = 20000000;

    void Activate() {
        tx.started_ = true;
        tx.started_ns_ = kStart;
    }
    bool Can(Producer producer, std::span<const Frame> frames,
             uint64_t sequence, uint64_t now, uint64_t generated = 0) {
        return tx.StoreCan(producer, frames, sequence,
                           generated ? generated : now, now);
    }
    bool Uart(std::span<const uint8_t> bytes, uint64_t now,
              uint64_t ttl = 200000000) {
        return tx.StoreUart(4, bytes, now, ttl, now);
    }
    auto Next(uint64_t now, unsigned slots = 8) {
        return tx.NextWork(now, slots);
    }
    void Run() {
        const Group chassis{2, 0x200}, gimbal{3, 0x200};
        assert(tx.ConfigureCan(Producer::kChassis, {&chassis, 1}, kTimeout));
        assert(!tx.ConfigureCan(Producer::kGimbal, {&chassis, 1}, kTimeout));
        const std::array<Group, 2> duplicates{gimbal, gimbal};
        assert(!tx.ConfigureCan(Producer::kGimbal, duplicates, kTimeout));
        assert(tx.ConfigureCan(Producer::kGimbal, {&gimbal, 1}, kTimeout));
        Activate();
        assert(!tx.ConfigureCan(Producer::kChassis, {&chassis, 1}, kTimeout));
        Frame frame{2, 0x200, {1}};
        assert(!Can(Producer::kGimbal, {&frame, 1}, 1, kStart));
        assert(Can(Producer::kChassis, {&frame, 1}, 1, kStart));
        std::array<uint8_t, 200> uart{};
        uart[0] = 42;
        assert(Uart(uart, kStart));
        uart[0] = 0;
        assert(Next(kStart).kind == Kind::kWait);
        assert(Can(Producer::kGimbal, {}, 1, kStart));
        frame.bytes[0] = 2;
        assert(Can(Producer::kChassis, {&frame, 1}, 2, kStart));
        assert(!Can(Producer::kChassis, {&frame, 1}, 2, kStart + 1));
        const std::array<Frame, 2> duplicate_frames{frame, frame};
        assert(!Can(Producer::kChassis, duplicate_frames, 3, kStart));
        auto work = Next(kStart);
        assert(work.kind == Kind::kCan && work.frames[0].bytes[0] == 2);
        work = Next(kStart);
        assert(work.kind == Kind::kCan && work.producer == Producer::kGimbal &&
               work.count == 0);
        assert(Next(kStart, 2).kind == Kind::kWait);
        work = Next(kStart);
        assert(work.kind == Kind::kUart && work.count == 96 &&
               work.bytes[0] == 42);
        assert(Next(kStart).kind == Kind::kWait);
        assert(Next(kStart + 12000000).count == 96);
        assert(Can(Producer::kGimbal, {}, 2, kStart + kTimeout - 1));
        work = Next(kStart + kTimeout);
        assert(work.kind == Kind::kExpired &&
               work.producer == Producer::kChassis);
        // 截止时间过后即使提交全新目标，也不能重新激活已失效来源。
        assert(!Can(Producer::kChassis, {&frame, 1}, 3, kStart + kTimeout));
    }
    static void SplittingAndQueue() {
        UsbTransmitFixture f;
        std::array<Group, 16> groups{};
        std::array<Frame, 16> frames{};
        for (size_t i = 0; i < groups.size(); ++i) {
            groups[i] = {2, static_cast<uint16_t>(0x100 + i)};
            frames[i] = {groups[i].bus, groups[i].id, {}};
        }
        assert(f.tx.ConfigureCan(Producer::kChassis, groups, kTimeout));
        f.Activate();
        assert(f.Can(Producer::kChassis, frames, 1, kStart));
        assert(f.Next(kStart).count == 6);
        assert(f.Next(kStart).count == 6);
        assert(f.Next(kStart).count == 4);
        assert(f.Next(kStart).kind == Kind::kWait);
        f.tx.Stop();
        assert(!f.Can(Producer::kChassis, frames, 2, kStart));

        UsbTransmitFixture u;
        u.Activate();
        const std::array<uint8_t, 200> bytes{};
        assert(u.Uart(bytes, kStart, 1));
        assert(u.Next(kStart + 1).kind ==
               Kind::kWait);  // 未开始的过期消息丢弃。
        assert(u.Uart(bytes, kStart + 1, 1));
        assert(u.Next(kStart + 1).count == 96);
        assert(u.Next(kStart + 12000001).count == 96);  // 已开始消息保持完整。
        assert(u.Next(kStart + 24000001).count == 8);
        assert(u.Next(kStart + 25000001).kind == Kind::kWait);
        for (unsigned i = 0; i < 16; ++i) {
            assert(u.Uart(bytes, kStart + 25000001));
        }
        assert(!u.Uart(bytes, kStart + 25000001));

        UsbTransmitFixture unprimed;
        assert(unprimed.tx.ConfigureCan(Producer::kChassis, {groups.data(), 1},
                                        kTimeout));
        unprimed.Activate();
        assert(unprimed.Uart(bytes, kStart));
        assert(unprimed.Next(kStart + kTimeout).kind == Kind::kExpired);
    }
};
}  // namespace dust::test

int main() {
    dust::test::UsbTransmitFixture fixture;
    fixture.Run();
    dust::test::UsbTransmitFixture::SplittingAndQueue();
    std::cout << "CAN ownership, latest batches, source deadlines and bounded "
                 "UART scheduling passed\n";
}
