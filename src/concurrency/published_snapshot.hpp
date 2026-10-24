/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <mutex>
#include <type_traits>

#include "priority_mutex.hpp"

namespace dust::concurrency {
// 单发布者、多读取者；优先级继承锁内只复制固定大小状态。
// Read 返回拥有所有权的副本，不暴露锁或内部引用，可能短暂阻塞。
template <typename T>
class PublishedSnapshot {
    static_assert(std::is_trivially_copyable_v<T>);

   public:
    PublishedSnapshot() = default;
    PublishedSnapshot(const PublishedSnapshot&) = delete;
    PublishedSnapshot& operator=(const PublishedSnapshot&) = delete;

    void Publish(const T& value) {
        const std::lock_guard lock(mutex_);
        value_ = value;
    }
    T Read() const {
        const std::lock_guard lock(mutex_);
        return value_;
    }

   private:
    mutable PriorityMutex mutex_;
    T value_{};
};
}  // namespace dust::concurrency
