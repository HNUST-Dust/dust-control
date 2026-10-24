/* SPDX-License-Identifier: Apache-2.0 */
#include <cassert>
#include <chrono>
#include <thread>

#include "chassis/runtime/realtime_loop.hpp"
#include "usb_link.hpp"
#include "usb_transmitter.hpp"
int main() {
    wbr::host::UartReceiveQueue store;
    wbr::host::UsbOptions options;
    options.realtime = false;
    wbr::host::UsbLink link(store, options);
    assert(link.Open());
    wbr::host::UsbTransmitOptions tx_options;
    tx_options.realtime = false;
    wbr::host::UsbTransmitter transmitter(link, tx_options);
    assert(transmitter.Start());
    const auto now = [] { return wbr::realtime::MonotonicTimeNs(); };
    const uint8_t bytes[] = {0xa5, 0x5a, 0x42};
    assert(!transmitter.SubmitUart(0, bytes, now()));
    assert(!transmitter.SubmitUart(7, bytes, now()));
    assert(!transmitter.SubmitUart(4, {}, now()));
    const std::array<uint8_t, 513> oversized{};
    assert(!transmitter.SubmitUart(4, oversized, now()));
    for (uint8_t channel = 4; channel <= 6; ++channel) {
        assert(transmitter.SubmitUart(channel, bytes, now()));
        wbr::host::UartReceiveChunk chunk{};
        bool received = false;
        for (unsigned i = 0; i < 10 && !received; ++i) {
            received = store.TakeUartChunk(channel, chunk);
            if (!received) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        assert(received && chunk.size == 3 && chunk.bytes[2] == 0x42);
    }
    transmitter.Stop();
    link.Close();
    assert(!transmitter.SubmitUart(4, bytes, now()));
}
