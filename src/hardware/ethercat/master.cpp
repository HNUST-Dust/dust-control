/* SPDX-License-Identifier: Apache-2.0 */
#include "master.hpp"

#include <algorithm>
#include <chrono>
#include <random>
#include <stdexcept>

#include "chassis/runtime/realtime_loop.hpp"
#include "wire.hpp"
namespace dust_control::ethercat {
EthercatMaster::EthercatMaster(PdoConfig config,
                               concurrency::ThreadOptions thread,
                               std::unique_ptr<Transport> transport)
    : config_(std::move(config)),
      thread_options_(thread),
      transport_(std::move(transport)) {
    concurrency::ValidateThreadOptions(thread_options_);
    const auto layout = BuildPdoLayout(config_);
    for (const auto& [name, capacity] : config_.adapters) {
        const auto& image = layout.adapters.at(name);
        boards_.push_back({name, capacity, image,
                           std::vector<uint8_t>(image.rx.bytes),
                           std::vector<uint8_t>(image.tx.bytes), false, false});
    }
    if (!transport_) {
        throw std::invalid_argument("EtherCAT transport is required");
    }
}
EthercatMaster::~EthercatMaster() {
    Stop();
}
void EthercatMaster::Attach(const std::string& name,
                            host::EthercatSession& session) {
    if (started_ || !config_.adapters.contains(name) ||
        !sessions_.emplace(name, &session).second) {
        throw std::logic_error("Invalid EtherCAT session attachment");
    }
}
bool EthercatMaster::Start() {
    if (started_ || stop_) {
        throw std::logic_error("EtherCAT master cannot restart");
    }
    started_ = true;
    if (sessions_.size() != boards_.size()) {
        Fail("Every configured EtherCAT adapter requires a session");
        return false;
    }
    for (auto& [name, session] : sessions_) {
        (void)name;
        session->transmitter().SealRegistration();
    }
    worker_ = std::thread([this] { Run(); });
    std::unique_lock lock(mutex_);
    changed_.wait_for(lock, std::chrono::seconds(6),
                      [this] { return ready_.load() || failed_.load(); });
    const bool good = ready_ && !failed_;
    lock.unlock();
    if (!good) {
        if (!failed_) {
            Fail("EtherCAT startup timed out");
        }
        Stop();
    }
    return good;
}
void EthercatMaster::Stop() {
    stop_ = true;
    if (worker_.joinable()) {
        worker_.join();
    }
    ready_ = false;
}
std::string EthercatMaster::error() const {
    const std::lock_guard lock(mutex_);
    return error_;
}
void EthercatMaster::Fail(const std::string& reason) {
    {
        const std::lock_guard lock(mutex_);
        if (error_.empty()) {
            error_ = reason;
        }
        failed_ = true;
    }
    changed_.notify_all();
}
void EthercatMaster::Run() {
    bool opened = false;
    uint64_t nonce = 0;
    uint32_t exchange = 0;
    MasterStatistics statistics;
    try {
        std::string reason;
        if (!transport_->Open(config_.master_index, boards_, reason)) {
            throw std::runtime_error(reason);
        }
        opened = true;
        std::random_device random;
        nonce = (uint64_t(random()) << 32) | random();
        if (!nonce) {
            nonce = 1;
        }
        if (!concurrency::ConfigureThread(thread_options_, reason)) {
            throw std::runtime_error(reason);
        }
        pthread_setname_np(pthread_self(), "dust-ethercat");
        const auto start = realtime::MonotonicTimeNs();
        auto release = start;
        while (!stop_ && !failed_) {
            release += 1000000;
            if (!realtime::SleepUntilNs(release, reason)) {
                throw std::runtime_error(reason);
            }
            const auto now = realtime::MonotonicTimeNs();
            ++statistics.cycles;
            statistics.max_lateness_ns =
                std::max(statistics.max_lateness_ns, now - release);
            statistics.missed_periods += (now - release) / 1000000;
            statistics_.Publish(statistics);
            if (now - release >= 20000000) {
                throw std::runtime_error(
                    "EtherCAT cycle deadline exceeded; new session required");
            }
            if (now - release >= 1000000) {
                release += ((now - release) / 1000000) * 1000000;
            }
            if (!transport_->Receive(boards_, reason)) {
                throw std::runtime_error(reason);
            }
            bool all_active = true, all_discovered = true;
            for (auto& board : boards_) {
                auto& session = *sessions_.at(board.name);
                if (board.operational && board.complete) {
                    if (!session.Receive(board.input, now, nonce)) {
                        throw std::runtime_error(board.name + ": " +
                                                 session.error());
                    }
                } else {
                    all_active = false;
                    all_discovered = false;
                    if (ready_) {
                        throw std::runtime_error(board.name +
                                                 ": EtherCAT OP/WKC lost");
                    }
                }
                all_active = all_active && session.active();
                all_discovered = all_discovered && session.boot_id() != 0;
            }
            if (!ready_ && all_active) {
                for (auto& [name, session] : sessions_) {
                    (void)name;
                    session->Activate(now);
                }
                {
                    const std::lock_guard lock(mutex_);
                    ready_ = true;
                }
                changed_.notify_all();
            }
            if (!ready_ && now - start >= 5000000000ULL) {
                throw std::runtime_error(
                    "EtherCAT OP/layout/session handshake timed out");
            }
            if (realtime::MonotonicTimeNs() - release >= 20000000) {
                throw std::runtime_error(
                    "EtherCAT receive exceeded dispatch deadline");
            }
            ++exchange;
            for (auto& board : boards_) {
                auto& session = *sessions_.at(board.name);
                if (ready_ && !session.healthy()) {
                    throw std::runtime_error(board.name +
                                             ": command source stopped");
                }
                const auto flags =
                    all_discovered ? wire::kArm : wire::kDiscover;
                if (!session.Encode(board.output, realtime::MonotonicTimeNs(),
                                    nonce, exchange, flags)) {
                    throw std::runtime_error(board.name + ": " +
                                             session.error());
                }
            }
            if (realtime::MonotonicTimeNs() - release >= 20000000) {
                throw std::runtime_error("EtherCAT dispatch deadline exceeded");
            }
            if (!transport_->Send(boards_, reason)) {
                throw std::runtime_error(reason);
            }
        }
    } catch (const std::exception& e) {
        Fail(e.what());
    }
    for (auto& [name, session] : sessions_) {
        (void)name;
        session->Fail(error().empty() ? "EtherCAT master stopped" : error());
    }
    // Clear every target before the first stop send, including on partial
    // startup. Every bus call remains on this thread. Slaves must independently
    // watchdog.
    if (opened) {
        try {
            std::string ignored;
            for (auto& board : boards_) {
                auto& session = *sessions_.at(board.name);
                session.Encode(board.output, realtime::MonotonicTimeNs(), nonce,
                               ++exchange, wire::kStop);
            }
            for (unsigned i = 0; i < 25; ++i) {
                transport_->Send(boards_, ignored);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                transport_->Receive(boards_, ignored);
            }
        } catch (...) { /* Local slave watchdog remains the final fallback. */
        }
    }
    transport_->Close();
    for (auto& [name, session] : sessions_) {
        (void)name;
        session->Fail(error().empty() ? "EtherCAT master stopped" : error());
    }
    ready_ = false;
    changed_.notify_all();
}
}  // namespace dust_control::ethercat
