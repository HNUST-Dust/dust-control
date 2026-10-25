/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <string>

namespace dust::concurrency {
// cpu=-1 不绑定；priority=0 为普通调度。realtime=false 禁用 FIFO，保留 CPU
// 配置。
struct ThreadOptions {
    bool realtime = true;
    int cpu = -1;
    int priority = 0;
};
void ValidateThreadOptions(const ThreadOptions& options);
// 由工作线程自身调用；只改变当前线程的亲和性/调度，FIFO 模式锁定进程内存。
bool ConfigureThread(const ThreadOptions& options, std::string& error);
}  // namespace dust::concurrency
