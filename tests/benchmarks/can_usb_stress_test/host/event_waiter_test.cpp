/* SPDX-License-Identifier: Apache-2.0 */
#include "event_waiter.hpp"

#include <time.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>

std::uint64_t NowNs() {
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    return std::uint64_t(now.tv_sec) * 1000000000ULL + now.tv_nsec;
}

int main() {
    int pipe_fds[2];
    if (pipe(pipe_fds) != 0) {
        return 1;
    }

    dust_control::benchmarks::stress_host::EventWaiter io_wait;
    if (!io_wait.Prepare({{pipe_fds[0], POLLIN, 0}}) ||
        !io_wait.Arm(NowNs() + 10000000000ULL, 1000000)) {
        return 2;
    }

    // 模拟已就绪 USB 描述符，必须立即唤醒而不等待计时器。
    const char byte = 'x';
    if (write(pipe_fds[1], &byte, 1) != 1) {
        return 3;
    }

    const auto io_result = io_wait.Wait();
    if (io_result.error != 0 || io_result.timer_expirations != 0) {
        return 4;
    }

    io_wait.Stop();
    close(pipe_fds[0]);
    close(pipe_fds[1]);

    dust_control::benchmarks::stress_host::EventWaiter timer_wait;
    if (!timer_wait.Prepare({}) ||
        !timer_wait.Arm(NowNs() - 5000000, 1000000)) {
        return 5;
    }

    // 内核到期次数应保留错过的 1 ms 周期，不能通过相对超时取整掩盖；
    // 测试不设置严格的耗时阈值。
    const auto timer_result = timer_wait.Wait();
    if (timer_result.error != 0 || timer_result.timer_expirations < 5) {
        return 6;
    }

    timer_wait.Stop();
    std::cout << "Descriptor readiness and absolute periodic timer both wake "
                 "correctly\n";
}
