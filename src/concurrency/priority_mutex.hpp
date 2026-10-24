/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <pthread.h>

#include <stdexcept>

namespace dust::concurrency {
// 实现 BasicLockable，供 std::lock_guard 表达短临界区的所有权。
class PriorityMutex {
   public:
    PriorityMutex() {
        pthread_mutexattr_t attributes;
        if (pthread_mutexattr_init(&attributes)) {
            throw std::runtime_error("mutex attributes");
        }
        const int protocol =
            pthread_mutexattr_setprotocol(&attributes, PTHREAD_PRIO_INHERIT);
        const int initialized =
            protocol ? protocol : pthread_mutex_init(&mutex_, &attributes);
        pthread_mutexattr_destroy(&attributes);
        if (initialized) {
            throw std::runtime_error("priority inheritance mutex");
        }
    }
    ~PriorityMutex() {
        pthread_mutex_destroy(&mutex_);
    }
    PriorityMutex(const PriorityMutex&) = delete;
    PriorityMutex& operator=(const PriorityMutex&) = delete;
    void lock() {
        pthread_mutex_lock(&mutex_);
    }
    void unlock() {
        pthread_mutex_unlock(&mutex_);
    }

   private:
    pthread_mutex_t mutex_{};
};
}  // namespace dust::concurrency
