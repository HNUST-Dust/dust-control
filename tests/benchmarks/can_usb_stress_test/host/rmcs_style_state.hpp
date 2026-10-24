/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <pthread.h>

#include <array>
#include <cstdint>
#include <mutex>
#include <stdexcept>

namespace stress_host {
// 使用优先级继承锁保护短时缓存，USB 负责更新。
// 控制线程复制六电机快照后先释放锁，再计算或提交 USB；
// 原始状态和两个时间戳必须一起复制。
class RmcsLatestState {
   public:
    struct Motor {
        bool valid = false;
        uint32_t received_us = 0;
        uint64_t host_ns = 0;
        std::array<unsigned char, 8> data{};
    };

    using Snapshot = std::array<Motor, 6>;
    RmcsLatestState() {
        pthread_mutexattr_t attributes;
        if (pthread_mutexattr_init(&attributes)) {
            throw std::runtime_error("mutexattr init failed");
        }
        int error =
            pthread_mutexattr_setprotocol(&attributes, PTHREAD_PRIO_INHERIT);
        if (!error) {
            error = pthread_mutex_init(&mutex_, &attributes);
        }
        pthread_mutexattr_destroy(&attributes);
        if (error) {
            throw std::runtime_error(
                "Priority-inheritance cache mutex unavailable");
        }
    }
    ~RmcsLatestState() {
        pthread_mutex_destroy(&mutex_);
    }

    RmcsLatestState(const RmcsLatestState &) = delete;
    RmcsLatestState &operator=(const RmcsLatestState &) = delete;
    void Put(unsigned motor, const Motor &sample) {
        pthread_mutex_lock(&mutex_);
        motors_[motor] = sample;
        pthread_mutex_unlock(&mutex_);
    }

    Snapshot Read() {
        pthread_mutex_lock(&mutex_);
        const auto snapshot = motors_;
        pthread_mutex_unlock(&mutex_);
        return snapshot;
    }

   private:
    pthread_mutex_t mutex_{};
    Snapshot motors_{};
};
}  // namespace stress_host
