/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "chassis_types.h"
#include <array>
#include <atomic>
#include <cstddef>
#include <thread>

namespace wbr::host {
struct TelemetrySample {
  modules::ChassisCycleInput input{};
  uint64_t monotonic_ns = 0;
  uint64_t sequence = 0;
  uint64_t missed_releases = 0;
  uint64_t control_duration_ns = 0;
  uint64_t queue_drops = 0;
  modules::ChassisControlState state{};
  bool simulated = false;
};
// Single control-thread producer, single non-RT consumer. Never waits or allocates.
class TelemetryQueue {
public:
  bool Push(const TelemetrySample &sample) {
    const auto write = write_.load(std::memory_order_relaxed);
    const auto next = (write + 1) % samples_.size();
    if (next == read_.load(std::memory_order_acquire))
      return false;
    samples_[write] = sample;
    write_.store(next, std::memory_order_release);
    return true;
  }
  bool Pop(TelemetrySample &sample) {
    const auto read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire))
      return false;
    sample = samples_[read];
    read_.store((read + 1) % samples_.size(), std::memory_order_release);
    return true;
  }
private:
  std::array<TelemetrySample, 64> samples_{};
  alignas(64) std::atomic<size_t> write_{0};
  alignas(64) std::atomic<size_t> read_{0};
  static_assert(std::atomic<size_t>::is_always_lock_free);
};

// Construct BEFORE elevating the control thread. JSON/network live only here.
class TelemetryPublisher {
public:
  TelemetryPublisher(unsigned port, int control_cpu, int usb_cpu,
                     unsigned foxglove_port = 0);
  ~TelemetryPublisher();
  bool Push(const TelemetrySample &sample) { return queue_.Push(sample); }
private:
  TelemetryQueue queue_;
  std::atomic<bool> stop_{false};
  std::thread worker_;
};
} // namespace wbr::host
