/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstdint>

namespace dust_control::benchmarks::stress_host {
// 在每个标称 1 ms 控制周期内提供可选的有界采样窗口。
// 等待期间仍处理 USB；反馈缺失不能超过绝对期限，也不能额外插入命令周期。
class FeedbackGate {
   public:
    static constexpr uint64_t kMaximumFeedbackAgeNs = 500000;
    struct Decision {
        bool send = false;
        bool expired = false;
        uint64_t waited_ns = 0;
    };

    void Begin(uint64_t nominal_ns, uint64_t observed_ns, unsigned grace_us) {
        active_ = true;
        observed_ns_ = observed_ns;
        deadline_ns_ = nominal_ns + uint64_t(grace_us) * 1000;
        wakeup_late_ns_ = observed_ns - nominal_ns;
    }

    Decision Evaluate(uint64_t now_ns, const std::array<uint32_t, 2> &rx_us,
                      const std::array<bool, 2> &valid,
                      const std::array<uint64_t, 2> &host_receive_ns) const {
        // 未使用过的样本也可能接近一个周期以前，
        // 因此同时检查样本是否更新和年龄，不能绕过等待窗口。
        const bool recent =
            now_ns >= host_receive_ns[0] && now_ns >= host_receive_ns[1] &&
            now_ns - host_receive_ns[0] <= kMaximumFeedbackAgeNs &&
            now_ns - host_receive_ns[1] <= kMaximumFeedbackAgeNs;
        const bool fresh =
            recent && valid[0] && valid[1] &&
            (!previous_valid_[0] || rx_us[0] != previous_us_[0]) &&
            (!previous_valid_[1] || rx_us[1] != previous_us_[1]);
        // 启动时不等待，设备可能需要命令才能产生反馈。
        const bool expired = now_ns >= deadline_ns_;
        return {!primed_ || fresh || expired, primed_ && !fresh && expired,
                now_ns - observed_ns_};
    }

    void Commit(const std::array<uint32_t, 2> &rx_us,
                const std::array<bool, 2> &valid = {true, true}) {
        previous_us_ = rx_us;
        previous_valid_ = valid;
        primed_ = true;
        active_ = false;
    }

    bool Active() const {
        return active_;
    }

    uint64_t DeadlineNs() const {
        return deadline_ns_;
    }

    uint64_t WakeupLateNs() const {
        return wakeup_late_ns_;
    }

   private:
    bool active_ = false;
    bool primed_ = false;
    std::array<uint32_t, 2> previous_us_{};
    std::array<bool, 2> previous_valid_{};
    uint64_t observed_ns_ = 0;
    uint64_t deadline_ns_ = 0;
    uint64_t wakeup_late_ns_ = 0;
};
}  // namespace dust_control::benchmarks::stress_host
