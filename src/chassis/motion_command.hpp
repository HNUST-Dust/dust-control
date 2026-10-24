/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dust::chassis {
struct MotionCommandOptions {
    bool external_commands = false;
    uint64_t timeout_ns = 200000000;
};

struct MotionCommand {
    double forward_m_s = 0;
    double yaw_rad_s = 0;
    double leg_rate_m_s = 0;
    uint64_t received_ns = 0;
};

// 外部指令单生产者，底盘线程单消费者；满时拒绝，不阻塞。
class MotionCommandQueue {
   public:
    // 复制完整指令；非法值或队列满返回 false，不代表控制器已经采用该指令。
    bool Push(const MotionCommand& command) {
        if (!std::isfinite(command.forward_m_s) ||
            !std::isfinite(command.yaw_rad_s) ||
            !std::isfinite(command.leg_rate_m_s)) {
            return false;
        }
        const auto write = write_.load(std::memory_order_relaxed);
        const auto next = (write + 1) % commands_.size();
        if (next == read_.load(std::memory_order_acquire)) {
            return false;
        }
        commands_[write] = command;
        write_.store(next, std::memory_order_release);
        return true;
    }
    // 取走当前可见记录并复制最新一条；空队列返回 false 且不修改输出。
    bool TakeLatest(MotionCommand& command) {
        bool found = false;
        auto read = read_.load(std::memory_order_relaxed);
        const auto write = write_.load(std::memory_order_acquire);
        while (read != write) {
            command = commands_[read];
            read = (read + 1) % commands_.size();
            found = true;
        }
        read_.store(read, std::memory_order_release);
        return found;
    }

   private:
    std::array<MotionCommand, 32> commands_{};
    alignas(64) std::atomic<size_t> write_{0};
    alignas(64) std::atomic<size_t> read_{0};
    static_assert(std::atomic<size_t>::is_always_lock_free);
};
}  // namespace dust::chassis
