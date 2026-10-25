/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <hardware/usb/protocol/usb_protocol.hpp>

#include "concurrency/priority_mutex.hpp"

namespace dust_control::host {
/** @brief 原始 UART 接收块；序号或 discontinuity 指示流不连续。 */
struct UartReceiveChunk {
    std::array<uint8_t, 40> bytes{};
    uint8_t size = 0;
    uint32_t sequence = 0;
    uint64_t receive_us = 0;
    bool discontinuity = false;
};

// 只保存原始 UART 块；设备状态由各 Decoder 的 Read() 提供。
class UartReceiveQueue {
   public:
    // 按逻辑通道复制原始块；满时丢最旧块并标记不连续，不动态分配。
    void PublishUartChunk(uint8_t channel, UartReceiveChunk chunk);
    // 加锁取走最早块；多消费者分摊记录，不是广播订阅。
    bool TakeUartChunk(uint8_t channel, UartReceiveChunk& chunk);
    uint64_t UartDroppedChunks(uint8_t channel);
    // 会话切换时清除原始队列和丢失计数。
    void ResetUartChunks();

   private:
    struct UartChunkQueue {
        std::array<UartReceiveChunk, 32> chunks{};
        size_t head = 0;
        size_t size = 0;
        uint64_t dropped = 0;
    };
    std::array<UartChunkQueue, dust_control::wire::kUartPortCount>
        uart_queues_{};
    dust_control::concurrency::PriorityMutex mutex_;
};
}  // namespace dust_control::host
