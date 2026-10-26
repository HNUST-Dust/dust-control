/* SPDX-License-Identifier: Apache-2.0 */
#include "ui_runtime.hpp"

#include <pthread.h>

#include <stdexcept>

namespace dust_control::ui {
UiRuntime::UiRuntime(UiDevices devices, concurrency::ThreadOptions thread)
    : devices_(devices), thread_(thread) {
    concurrency::ValidateThreadOptions(thread_);
    if (bool(devices_.referee) != devices_.uart.has_value()) {
        throw std::invalid_argument("Incomplete UI device interfaces");
    }
}
UiRuntime::~UiRuntime() {
    Stop();
}
void UiRuntime::Start() {
    const std::lock_guard lock(mutex_);
    if (started_ || stop_) {
        throw std::logic_error("ui runtime cannot be restarted");
    }
    started_ = true;
    worker_ = std::thread([this] { Run(); });
}
void UiRuntime::Stop() {
    {
        const std::lock_guard lock(mutex_);
        stop_ = true;
    }
    changed_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    finished_ = true;
}
void UiRuntime::Run() {
    pthread_setname_np(pthread_self(), "dust-ui");
    try {
        if (!concurrency::ConfigureThread(thread_, error_)) {
            finished_ = true;
            return;
        }
        std::unique_lock lock(mutex_);
        changed_.wait(lock, [this] { return stop_; });
    } catch (const std::exception& exception) {
        error_ = exception.what();
    }
    finished_ = true;
}
}  // namespace dust_control::ui
