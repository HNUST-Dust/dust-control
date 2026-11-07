/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include "concurrency/thread.hpp"

namespace dust_control::ui {
// 当前仅建立独立线程与生命周期；等待停止，不生成命令或维持虚假心跳。
class UiRuntime {
   public:
    explicit UiRuntime(concurrency::ThreadOptions thread = {});
    ~UiRuntime();
    void Start();
    void Stop();
    bool finished() const {
        return finished_.load();
    }
    // 仅在 finished() 或 Stop() 后读取。
    const std::string& error() const {
        return error_;
    }

   private:
    void Run();
    const concurrency::ThreadOptions thread_;
    std::mutex mutex_;
    std::condition_variable changed_;
    bool started_ = false;
    bool stop_ = false;
    std::atomic<bool> finished_{false};
    std::string error_;
    std::thread worker_;
};
}  // namespace dust_control::ui
