/* SPDX-License-Identifier: Apache-2.0 */
#include "echo_packet.hpp"

#include <algorithm>
#include <iostream>

int main() {
    // Python/固件共用样本：magic、序列号 17、前次 RTT 2048（小端），
    // 后接恰好 52 字节的载荷，数值依次为 17..68。
    const unsigned char header[] = {0x53, 0x52, 0x54, 0x53, 17, 0,
                                    0,    0,    0,    8,    0,  0};
    const auto packet =
        dust_control::benchmarks::stress_host::EncodePacket(17, 2048);
    if (!std::equal(std::begin(header), std::end(header), packet.begin())) {
        return 1;
    }

    for (unsigned index = 12; index < 64; ++index) {
        if (packet[index] != index + 5) {
            return 2;
        }
    }

    const auto wrapped = dust_control::benchmarks::stress_host::EncodePacket(
        0xffffffffU, 0x12345678U);
    if (dust_control::benchmarks::stress_host::ReadU32(wrapped.data() + 4) !=
            0xffffffffU ||
        dust_control::benchmarks::stress_host::ReadU32(wrapped.data() + 8) !=
            0x12345678U ||
        wrapped[12] != 255) {
        return 3;
    }

    std::cout << "Packet matches Python/firmware wire contract\n";
}
