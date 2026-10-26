/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <condition_variable>
#include <mutex>
#include <thread>

#include "concurrency/thread.hpp"
#include "session.hpp"
#include "transport.hpp"
namespace dust_control::ethercat {
struct MasterStatistics {
    uint64_t cycles = 0, missed_periods = 0, max_lateness_ns = 0;
};
class EthercatMaster {
   public:
    EthercatMaster(PdoConfig config, concurrency::ThreadOptions thread,
                   std::unique_ptr<Transport> transport = CreateIghTransport());
    ~EthercatMaster();
    void Attach(const std::string& name, host::EthercatSession& session);
    bool Start();
    void Stop();
    bool healthy() const {
        return ready_.load() && !failed_.load() && !stop_.load();
    }
    std::string error() const;
    MasterStatistics statistics() const {
        return statistics_.Read();
    }

   private:
    void Run();
    void Fail(const std::string& reason);
    const PdoConfig config_;
    const concurrency::ThreadOptions thread_options_;
    std::unique_ptr<Transport> transport_;
    std::map<std::string, host::EthercatSession*> sessions_;
    std::vector<BoardImage> boards_;
    std::thread worker_;
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    std::atomic<bool> stop_{false}, failed_{false}, ready_{false};
    bool started_ = false;
    std::string error_;
    concurrency::PublishedSnapshot<MasterStatistics> statistics_;
};
}  // namespace dust_control::ethercat
