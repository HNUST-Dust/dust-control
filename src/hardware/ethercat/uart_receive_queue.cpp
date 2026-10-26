/* SPDX-License-Identifier: Apache-2.0 */
#include "uart_receive_queue.hpp"

#include <mutex>

namespace dust_control::host {
void UartReceiveQueue::PublishUartChunk(uint8_t channel,
                                        UartReceiveChunk chunk) {
    if (channel < 1 || channel > 6) {
        return;
    }
    const std::lock_guard lock(mutex_);
    auto& queue = uart_queues_[channel - 1];
    if (queue.size == queue.chunks.size()) {
        queue.head = (queue.head + 1) % queue.chunks.size();
        --queue.size;
        ++queue.dropped;
        queue.chunks[queue.head].discontinuity = true;
    }
    queue.chunks[(queue.head + queue.size) % queue.chunks.size()] = chunk;
    ++queue.size;
}

bool UartReceiveQueue::TakeUartChunk(uint8_t channel, UartReceiveChunk& chunk) {
    if (channel < 1 || channel > 6) {
        return false;
    }
    const std::lock_guard lock(mutex_);
    auto& queue = uart_queues_[channel - 1];
    const bool found = queue.size != 0;
    if (found) {
        chunk = queue.chunks[queue.head];
        queue.head = (queue.head + 1) % queue.chunks.size();
        --queue.size;
    }
    return found;
}

uint64_t UartReceiveQueue::UartDroppedChunks(uint8_t channel) {
    if (channel < 1 || channel > 6) {
        return 0;
    }
    const std::lock_guard lock(mutex_);
    const auto count = uart_queues_[channel - 1].dropped;
    return count;
}

void UartReceiveQueue::ResetUartChunks() {
    const std::lock_guard lock(mutex_);
    uart_queues_ = {};
}
}  // namespace dust_control::host
