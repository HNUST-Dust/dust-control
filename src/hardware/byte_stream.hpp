/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace dust_control::devices {
// 解析器独占的有界拼帧缓冲；不维护通道或设备状态。
template <size_t Capacity>
struct ByteStream {
    std::array<uint8_t, Capacity> bytes{};
    size_t size = 0;

    void DiscardPrefix(size_t count) {
        if (count >= size) {
            size = 0;
            return;
        }
        std::memmove(bytes.data(), bytes.data() + count, size - count);
        size -= count;
    }
    void Append(uint8_t byte) {
        if (size == Capacity) {
            DiscardPrefix(1);
        }
        bytes[size++] = byte;
    }
};
}  // namespace dust_control::devices
