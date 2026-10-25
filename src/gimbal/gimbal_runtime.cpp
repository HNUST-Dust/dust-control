/* SPDX-License-Identifier: Apache-2.0 */
#include "gimbal_runtime.hpp"

#include <pthread.h>

#include <stdexcept>

namespace dust_control::gimbal {
GimbalRuntime::GimbalRuntime(GimbalDevices devices,
                             concurrency::ThreadOptions thread)
    : devices_(devices), thread_(thread) {
    concurrency::ValidateThreadOptions(thread_);
    if ((devices_.motors && !devices_.can) || (devices_.imu && !devices_.can)) {
        throw std::invalid_argument("Incomplete gimbal device interfaces");
    }
}
GimbalRuntime::~GimbalRuntime() {
    Stop();
}
void GimbalRuntime::Start() {
    const std::lock_guard lock(mutex_);
    if (started_ || stop_) {
        throw std::logic_error("gimbal runtime cannot be restarted");
    }
    started_ = true;
    worker_ = std::thread([this] { Run(); });
}
void GimbalRuntime::Stop() {
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
void GimbalRuntime::Run() {
    pthread_setname_np(pthread_self(), "dust-gimbal");
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
}  // namespace dust_control::gimbal
