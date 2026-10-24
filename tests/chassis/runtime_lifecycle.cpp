/* SPDX-License-Identifier: Apache-2.0 */
#include <cassert>
#include <chrono>
#include <stdexcept>
#include <thread>

#include "chassis/chassis_runtime.hpp"
#include "chassis/runtime/realtime_loop.hpp"
#include "hardware/usb/usb_session.hpp"

int main() {
    using namespace dust::chassis;
    ChassisRuntimeOptions options;
    options.realtime = false;
    options.seconds = .01;
    ChassisRuntime offline(options, {}, {});
    offline.Start();
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!offline.finished() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    assert(offline.finished());
    offline.Stop();
    assert(offline.error().empty());
    wbr::host::TelemetrySample sample;
    assert(offline.TakeObservation(sample) && sample.simulated);
    bool rejected = false;
    try {
        offline.Start();
    } catch (const std::logic_error&) {
        rejected = true;
    }
    assert(rejected);

    wbr::host::UsbOptions usb_options;
    usb_options.realtime = false;
    wbr::host::UsbSession usb(usb_options, {}, {});
    const auto& link = usb.link();
    {
        ChassisDevices devices{&link.motor_decoder(), &link.sbus_decoder(),
                               &link.hi91_decoder(), &usb.transmitter()};
        ChassisRuntime chassis(options, {}, {}, devices);
        chassis.Stop();
    }
    // 底盘 Stop/析构仅回收自己的线程，不能停止借用的发送器或关闭共享链路。
    assert(usb.Start());
    assert(usb.transmitter().SubmitCan(wbr::host::CanProducer::kChassis, {}, 1,
                                       wbr::realtime::MonotonicTimeNs()));
    usb.Stop();
    assert(!usb.healthy());
    rejected = false;
    try {
        usb.Start();
    } catch (const std::logic_error&) {
        rejected = true;
    }
    assert(rejected);

    wbr::host::UsbSession stopped(usb_options, {}, {});
    stopped.Stop();
    rejected = false;
    try {
        stopped.Start();
    } catch (const std::logic_error&) {
        rejected = true;
    }
    assert(rejected);
}
