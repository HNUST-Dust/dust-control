/* SPDX-License-Identifier: Apache-2.0 */
#include <chrono>
#include <iostream>
#include <thread>

#include "../support/motor_fixture.hpp"
#include "chassis/runtime/realtime_loop.hpp"
#include "usb_link.hpp"
#include "usb_transmitter.hpp"
int main() {
    wbr::host::UartReceiveQueue sensors;
    wbr::host::UsbOptions options;
    options.realtime = false;
    for (unsigned session = 0; session < 5; ++session) {
        wbr::host::UsbLink link(sensors, options, dust::test::MotorFixture());
        if (!link.Open()) {
            std::cerr << link.error();
            return 1;
        }
        wbr::host::UsbTransmitOptions tx_options;
        tx_options.realtime = false;
        wbr::host::UsbTransmitter transmitter(link, tx_options);
        const wbr::host::CanCommandGroup group{2, 0x200};
        if (!transmitter.ConfigureCan(wbr::host::CanProducer::kChassis,
                                      {&group, 1}, 20000000) ||
            !transmitter.Start()) {
            return 5;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
        // 新 Hello 清空旧状态，注入的旧会话报文必须继续被拒绝。
        if (link.motor_decoder().Read()[0].valid) {
            return 2;
        }
        for (unsigned tick = 0; tick < 3; ++tick) {
            if (!transmitter.SubmitCan(wbr::host::CanProducer::kChassis, {},
                                       tick + 1,
                                       wbr::realtime::MonotonicTimeNs())) {
                return 3;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        if (!link.motor_decoder().Read()[0].valid) {
            return 4;
        }
        transmitter.Stop();
        link.Close();
    }

    std::cout << "Five USB sessions: state reset, stale-session rejection, new "
                 "sequence and drain passed\n";
}
