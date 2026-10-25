/* SPDX-License-Identifier: Apache-2.0 */
#include <cassert>
#include <chrono>
#include <stdexcept>
#include <thread>

#include "../support/motor_fixture.hpp"
#include "chassis/chassis_runtime.hpp"
#include "chassis/runtime/realtime_loop.hpp"
#include "hardware/usb/usb_session.hpp"

int main() {
    using namespace dust_control::chassis;
    ChassisRuntimeOptions options;
    options.thread.realtime = false;
    ChassisRuntime offline(options, {}, {});
    offline.Start();
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    dust_control::chassis::ChassisObservation sample;
    bool observed = false;
    while (!(observed = offline.TakeObservation(sample)) &&
           !offline.finished() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    assert(observed && sample.simulated);
    offline.Stop();
    assert(offline.finished());
    assert(offline.error().empty());
    bool rejected = false;
    try {
        offline.Start();
    } catch (const std::logic_error&) {
        rejected = true;
    }
    assert(rejected);

    dust_control::host::UsbOptions usb_options;
    usb_options.realtime = false;
    dust_control::host::UsbSession usb(usb_options,
                                       dust_control::test::MotorFixture(), {});
    const auto& link = usb.link();
    {
        ChassisDevices devices;
        devices.motor_adapters.push_back({"base", &link.motor_decoder(),
                                          &usb.transmitter(),
                                          dust_control::test::MotorFixture()});
        devices.remote = &link.wfly_sbus_decoder();
        devices.imu = &link.hi91_decoder();
        ChassisRuntime chassis(options, {}, {}, devices);
        chassis.Stop();
    }
    // 底盘 Stop/析构仅回收自己的线程，不能停止借用的发送器或关闭共享链路。
    assert(usb.Start());
    assert(usb.transmitter().SubmitCan(
        dust_control::host::CanProducer::kChassis, {}, 1,
        dust_control::realtime::MonotonicTimeNs()));
    usb.Stop();
    assert(!usb.healthy());
    rejected = false;
    try {
        usb.Start();
    } catch (const std::logic_error&) {
        rejected = true;
    }
    assert(rejected);

    dust_control::host::UsbSession stopped(usb_options, {}, {});
    stopped.Stop();
    rejected = false;
    try {
        stopped.Start();
    } catch (const std::logic_error&) {
        rejected = true;
    }
    assert(rejected);
}
