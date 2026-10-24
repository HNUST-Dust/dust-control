/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <poll.h>

#include <cstdint>
#include <vector>

namespace stress_host {

struct WaitResult {
    int error = 0;  // 负 errno；-EINTR 由调用者处理停止信号。
    std::uint64_t timer_expirations = 0;
};

// 内核 timerfd 在绝对命令期限唤醒 poll，poll 保持无限等待。
// 由此避免 libusb 的毫秒超时取整影响发送节奏。
// USB 描述符仅借用，计时器描述符由本类持有。
class EventWaiter {
   public:
    EventWaiter() = default;
    ~EventWaiter();
    EventWaiter(const EventWaiter &) = delete;
    EventWaiter &operator=(const EventWaiter &) = delete;
    bool Prepare(std::vector<pollfd> sources);
    bool Arm(std::uint64_t first_deadline_ns, std::uint64_t period_ns);
    WaitResult Wait();
    void Stop();

   private:
    int timer_fd_ = -1;
    std::vector<pollfd> sources_;
};

}  // namespace stress_host
