/* SPDX-License-Identifier: Apache-2.0 */
#include "thread.hpp"

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace dust_control::concurrency {
void ValidateThreadOptions(const ThreadOptions& options) {
    if (options.cpu < -1 || options.cpu >= CPU_SETSIZE ||
        options.priority < 0 || options.priority > 99 ||
        (options.priority > 0 && options.cpu < 0)) {
        throw std::invalid_argument("Invalid thread CPU or priority");
    }
}
bool ConfigureThread(const ThreadOptions& options, std::string& error) {
    ValidateThreadOptions(options);
    const bool fifo = options.realtime && options.priority > 0;
    if (fifo && mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        error = std::string("mlockall: ") + std::strerror(errno);
        return false;
    }
    if (options.realtime && options.cpu >= 0) {
        cpu_set_t cpus;
        CPU_ZERO(&cpus);
        CPU_SET(options.cpu, &cpus);
        const int result =
            pthread_setaffinity_np(pthread_self(), sizeof(cpus), &cpus);
        if (result != 0) {
            error =
                std::string("pthread_setaffinity_np: ") + std::strerror(result);
            return false;
        }
    }
    sched_param parameters{};
    parameters.sched_priority = fifo ? options.priority : 0;
    const int result = pthread_setschedparam(
        pthread_self(), fifo ? SCHED_FIFO : SCHED_OTHER, &parameters);
    if (result != 0) {
        error = std::string("pthread_setschedparam: ") + std::strerror(result);
        return false;
    }
    return true;
}
}  // namespace dust_control::concurrency
