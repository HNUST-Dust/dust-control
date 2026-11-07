/* SPDX-License-Identifier: Apache-2.0 */
#include "commands.hpp"

#include <algorithm>
#include <mutex>

#include "concurrency/realtime_loop.hpp"

namespace dust_control::host {
std::optional<devices::CanTransmitBatch> MotorCommandEncoder::Encode(
    const devices::MotorTargets& targets,
    const devices::MotorTargets& idle_targets) const {
    auto complete_targets = targets;
    for (size_t i = 0; i < complete_targets.size(); ++i) {
        if (complete_targets[i].mode == devices::MotorMode::kNone) {
            complete_targets[i] = idle_targets[i];
        }
    }
    auto batch = encoder_.Encode(complete_targets);
    if (!batch || batch->count != 6) {
        return std::nullopt;
    }
    return batch;
}
bool EthercatTransmitter::ConfigureCan(std::span<const CanCommandGroup> groups,
                                       uint64_t timeout_ns) {
    const std::lock_guard lock(mutex_);
    if (started_ || registration_closed_ || registered_ ||
        groups.size() != groups_.size() || !timeout_ns ||
        timeout_ns > kCommandTimeoutUs * 1000ULL) {
        return false;
    }
    for (const auto& route : ethercat::pdo::kRoutes) {
        if (std::count(groups.begin(), groups.end(),
                       CanCommandGroup{route.bus, route.command}) != 1) {
            return false;
        }
    }
    std::copy(groups.begin(), groups.end(), groups_.begin());
    timeout_ns_ = timeout_ns;
    registered_ = true;
    return true;
}
void EthercatTransmitter::SealRegistration() {
    const std::lock_guard lock(mutex_);
    registration_closed_ = true;
}
void EthercatTransmitter::Activate(uint64_t now_ns) {
    const std::lock_guard lock(mutex_);
    if (!started_ && !stopped_) {
        started_ = true;
        started_ns_ = now_ns;
        active_ = true;
    }
}
void EthercatTransmitter::Fail(const std::string& reason) {
    const std::lock_guard lock(mutex_);
    if (error_.empty()) {
        error_ = reason;
    }
    stopped_ = true;
    active_ = false;
}
std::string EthercatTransmitter::error() const {
    const std::lock_guard lock(mutex_);
    return error_;
}
bool EthercatTransmitter::SubmitCan(std::span<const devices::CanFrame> frames,
                                    uint64_t sequence, uint64_t generated_ns) {
    const auto now = realtime::MonotonicTimeNs();
    const std::lock_guard lock(mutex_);
    if (!healthy() || !registered_ || frames.size() != groups_.size() ||
        !sequence || sequence <= sequence_ || generated_ns > now ||
        generated_ns < generated_ns_ || now - generated_ns >= timeout_ns_ ||
        now - (batch_.valid ? generated_ns_ : started_ns_) >= timeout_ns_) {
        return false;
    }
    for (const auto& group : groups_) {
        if (std::count_if(frames.begin(), frames.end(), [&](const auto& frame) {
                return frame.bus == group.bus && frame.id == group.id;
            }) != 1) {
            return false;
        }
    }
    sequence_ = sequence;
    generated_ns_ = generated_ns;
    batch_.valid = true;
    batch_.sequence = static_cast<uint32_t>(sequence);
    batch_.count = frames.size();
    std::copy(frames.begin(), frames.end(), batch_.frames.begin());
    return true;
}
bool EthercatTransmitter::ReadBatch(uint64_t now, Batch& batch) {
    const std::lock_guard lock(mutex_);
    batch = {};
    if (!registered_) {
        return true;
    }
    const auto time = batch_.valid ? generated_ns_ : started_ns_;
    if (now < time || now - time >= timeout_ns_) {
        return false;
    }
    batch = batch_;
    return true;
}
}  // namespace dust_control::host
