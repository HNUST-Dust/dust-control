/* SPDX-License-Identifier: Apache-2.0 */
#include <cassert>
#include <chrono>
#include <thread>

#include "chassis/runtime/realtime_loop.hpp"
#include "usb_link.hpp"
#include "usb_transmitter.hpp"

int main() {
    using namespace wbr::host;
    UartReceiveQueue queue;
    UsbOptions usb_options;
    usb_options.realtime = false;
    UsbLink link(queue, usb_options);
    UsbTransmitOptions tx_options;
    tx_options.realtime = false;
    UsbTransmitter tx(link, tx_options);
    const CanCommandGroup chassis{2, 0x200}, gimbal{3, 0x200};
    assert(tx.ConfigureCan(CanProducer::kChassis, {&chassis, 1}, 20000000));
    assert(tx.ConfigureCan(CanProducer::kGimbal, {&gimbal, 1}, 20000000));
    assert(link.Open());
    assert(tx.Start());
    const auto now = wbr::realtime::MonotonicTimeNs();
    assert(tx.SubmitCan(CanProducer::kChassis, {}, 1, now));
    // 云台与 UI 持续更新，底盘停止提交。会话必须仍因底盘超时而停止。
    std::thread gimbal_thread([&] {
        uint64_t sequence = 0;
        while (tx.healthy()) {
            if (!tx.SubmitCan(CanProducer::kGimbal, {}, ++sequence,
                              wbr::realtime::MonotonicTimeNs())) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    std::thread ui_thread([&] {
        const uint8_t bytes[] = {0xa5, 0x5a, 0x42};
        while (tx.healthy()) {
            tx.SubmitUart(4, bytes, wbr::realtime::MonotonicTimeNs());
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (tx.healthy() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    assert(!tx.healthy());
    tx.Stop();
    gimbal_thread.join();
    ui_thread.join();
    assert(tx.error().find("chassis") != std::string::npos);
    assert(!tx.SubmitCan(CanProducer::kChassis, {}, 2,
                         wbr::realtime::MonotonicTimeNs()));
    link.Close();
}
