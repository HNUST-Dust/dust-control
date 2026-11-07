/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <atomic>
#include <optional>
#include <span>
#include <string>

#include "concurrency/priority_mutex.hpp"
#include "hardware/motors/motor_protocol.hpp"
#include "pdo_config.hpp"

namespace dust_control::host {
inline constexpr uint32_t kCommandTimeoutUs = 20000;
struct CanCommandGroup {
    uint8_t bus = 0;
    uint16_t id = 0;
    bool operator==(const CanCommandGroup&) const = default;
};
// 只按当前周期目标补齐 DTG1 六电机批次，不读取控制状态或历史目标。
class MotorCommandEncoder {
   public:
    explicit MotorCommandEncoder(const devices::MotorConfig& motors)
        : encoder_(motors) {}
    std::optional<devices::CanTransmitBatch> Encode(
        const devices::MotorTargets& targets,
        const devices::MotorTargets& idle_targets) const;

   private:
    devices::MotorEncoder encoder_;
};
// 业务线程提交完整批次，唯一主站线程读取；注册在启动前完成。
class EthercatTransmitter {
   public:
    bool ConfigureCan(std::span<const CanCommandGroup> groups,
                      uint64_t timeout_ns);
    bool SubmitCan(std::span<const devices::CanFrame> frames, uint64_t sequence,
                   uint64_t generated_ns);
    bool healthy() const {
        return active_.load() && !stopped_.load();
    }
    std::string error() const;
    void Activate(uint64_t now_ns);
    void SealRegistration();
    void Fail(const std::string& reason);
    struct Batch {
        std::array<devices::CanFrame, 6> frames{};
        size_t count = 0;
        uint32_t sequence = 0;
        bool valid = false;
    };
    bool ReadBatch(uint64_t now_ns, Batch& batch);

   private:
    mutable concurrency::PriorityMutex mutex_;
    std::array<CanCommandGroup, 6> groups_{};
    uint64_t timeout_ns_ = 0, generated_ns_ = 0, sequence_ = 0;
    Batch batch_;
    uint64_t started_ns_ = 0;
    bool registered_ = false, started_ = false, registration_closed_ = false;
    std::atomic<bool> active_{false}, stopped_{false};
    std::string error_;
};
}  // namespace dust_control::host
