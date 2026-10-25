/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "chassis_types.h"

namespace dust_control::chassis {
struct ChassisObservation {
    dust_control::modules::ChassisCycleInput input{};
    uint64_t monotonic_ns = 0;
    uint64_t sequence = 0;
    uint64_t missed_releases = 0;
    uint64_t control_duration_ns = 0;
    // 队列满时累计丢样，随下一次成功入队的观测发布。
    uint64_t queue_drops = 0;
    dust_control::modules::ChassisControlState state{};
    bool simulated = false;
};

// 控制线程单生产者、非实时线程单消费者；入队和出队均不等待、不分配内存。
class ChassisObservationQueue {
   public:
    static constexpr size_t kCapacity = 63;
    // 复制样本入队；满队列返回 false，不等待消费者。
    bool Push(const ChassisObservation &sample) {
        const auto write = write_.load(std::memory_order_relaxed);
        const auto next = (write + 1) % samples_.size();
        if (next == read_.load(std::memory_order_acquire)) {
            return false;
        }
        samples_[write] = sample;
        write_.store(next, std::memory_order_release);
        return true;
    }

    // 复制一个样本到调用者，空队列返回 false。
    bool Pop(ChassisObservation &sample) {
        const auto read = read_.load(std::memory_order_relaxed);
        if (read == write_.load(std::memory_order_acquire)) {
            return false;
        }
        sample = samples_[read];
        read_.store((read + 1) % samples_.size(), std::memory_order_release);
        return true;
    }

   private:
    std::array<ChassisObservation, kCapacity + 1> samples_{};
    alignas(64) std::atomic<size_t> write_{0};
    alignas(64) std::atomic<size_t> read_{0};
    static_assert(std::atomic<size_t>::is_always_lock_free);
};

}  // namespace dust_control::chassis
