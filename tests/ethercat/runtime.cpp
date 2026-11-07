/* SPDX-License-Identifier: Apache-2.0 */
#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <thread>

#include "concurrency/realtime_loop.hpp"
#include "dust_control.hpp"
#include "hardware/ethercat/device_pdo.hpp"
#include "hardware/ethercat/master.hpp"
#include "hardware/ethercat/wire.hpp"

namespace ec = dust_control::ethercat;
namespace host = dust_control::host;
namespace dw = ec::device;
namespace w = ec::wire;
using namespace std::chrono_literals;

ec::PdoConfig Config(unsigned count = 1) {
    ec::PdoConfig config;
    for (unsigned i = 0; i < count; ++i) {
        ec::AdapterPdoConfig board;
        board.position = i;
        config.adapters.emplace("board" + std::to_string(i), board);
    }
    return config;
}
void Commands() {
    host::EthercatTransmitter tx;
    std::array<host::CanCommandGroup, 6> groups{};
    std::array<dust_control::devices::CanFrame, 6> frames{};
    for (size_t i = 0; i < groups.size(); ++i) {
        groups[i] = {ec::pdo::kRoutes[i].bus, ec::pdo::kRoutes[i].command};
        frames[i] = {groups[i].bus, groups[i].id, {uint8_t(i)}};
    }
    assert(!tx.ConfigureCan({groups.data(), 5}, 20000000));
    auto bad = groups;
    bad[5] = bad[0];
    assert(!tx.ConfigureCan(bad, 20000000));
    assert(tx.ConfigureCan(groups, 20000000));
    assert(!tx.ConfigureCan(groups, 20000000));
    const auto now = dust_control::realtime::MonotonicTimeNs();
    tx.Activate(now);
    assert(tx.SubmitCan(frames, 1, now));
    assert(!tx.SubmitCan(frames, 1, now));
    assert(!tx.SubmitCan({}, 2, now));
    assert(!tx.SubmitCan(frames, 2, now + 100000000));
    auto duplicate = frames;
    duplicate[5] = duplicate[0];
    assert(!tx.SubmitCan(duplicate, 2, now));
    host::EthercatTransmitter::Batch batch;
    assert(tx.ReadBatch(now + 1000000, batch));
    assert(batch.valid && batch.count == 6 && batch.sequence == 1);
    assert(!tx.ReadBatch(now + 20000000, batch));
    tx.Fail("test stop");
    assert(!tx.SubmitCan(frames, 2, now));
}
struct Observation {
    std::atomic<unsigned> stops{0}, receives{0};
    std::atomic<bool> closed{false}, wrong_thread{false};
};
class FakeTransport final : public ec::Transport {
   public:
    FakeTransport(std::shared_ptr<Observation> observation, unsigned fault)
        : observation_(observation), fault_(fault) {}
    bool Open(uint32_t, std::vector<ec::BoardImage>&, std::string&) override {
        owner_ = std::this_thread::get_id();
        return true;
    }
    bool Receive(std::vector<ec::BoardImage>& boards, std::string&) override {
        CheckThread();
        const auto cycle = ++observation_->receives;
        if (fault_ == 5 && cycle == 40) {
            std::this_thread::sleep_for(22ms);
        }
        for (auto& board : boards) {
            board.operational = true;
            board.complete = !(fault_ == 1 && cycle > 40);
            dw::Payload token{};
            w::Put32(token.data(), nonce_);
            token[4] = fault_ == 4 ? 3 : 2;
            const bool reset = fault_ == 2 && cycle > 40;
            const auto record = dw::Encode(
                !armed_ || reset ? dw::Type::Ready : dw::Type::Status,
                uint8_t(cycle), token);
            std::fill(board.input.begin(), board.input.end(), 0);
            std::copy(record.begin(), record.end(), board.input.begin());
            if (fault_ == 3 && cycle > 40) {
                board.input[0] = 0xff;  // 活动会话中损坏的电机页。
            }
        }
        return true;
    }
    bool Send(const std::vector<ec::BoardImage>& boards,
              std::string&) override {
        CheckThread();
        bool all_stop = true;
        for (const auto& board : boards) {
            dw::View record{};
            assert(dw::Decode(dw::Direction::Rx, board.output.data(),
                              dw::kBytes, record));
            nonce_ = w::U32(record.payload.data());
            assert(nonce_);
            armed_ |= record.type == dw::Type::Arm;
            all_stop &= record.type == dw::Type::Stop;
        }
        if (all_stop) {
            ++observation_->stops;
        }
        return true;
    }
    void Close() override {
        CheckThread();
        observation_->closed = true;
    }

   private:
    void CheckThread() {
        if (owner_ != std::this_thread::get_id()) {
            observation_->wrong_thread = true;
        }
    }
    std::shared_ptr<Observation> observation_;
    unsigned fault_;
    uint32_t nonce_ = 0;
    bool armed_ = false;
    std::thread::id owner_;
};
void MasterLifecycle(unsigned count, unsigned fault) {
    const auto config = Config(count);
    const auto layout = ec::BuildPdoLayout(config);
    auto observation = std::make_shared<Observation>();
    std::map<std::string, std::unique_ptr<host::EthercatSession>> sessions;
    ec::EthercatMaster master(
        config, {false, -1, 0},
        std::make_unique<FakeTransport>(observation, fault));
    for (const auto& [name, capacity] : config.adapters) {
        (void)capacity;
        auto session =
            std::make_unique<host::EthercatSession>(layout.adapters.at(name));
        master.Attach(name, *session);
        sessions.emplace(name, std::move(session));
    }
    const bool started = master.Start();
    if (fault == 4) {
        assert(!started && !master.error().empty());
    } else {
        assert(started);
        if (fault) {
            const auto deadline = std::chrono::steady_clock::now() + 1s;
            while (master.healthy() &&
                   std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(1ms);
            }
            assert(!master.healthy() && !master.error().empty());
        }
    }
    master.Stop();
    assert(observation->closed && !observation->wrong_thread &&
           observation->stops == 25);
    assert(master.statistics().cycles > 0);
    for (const auto& [name, session] : sessions) {
        (void)name;
        assert(!session->healthy());
    }
    bool rejected = false;
    try {
        master.Start();
    } catch (const std::logic_error&) {
        rejected = true;
    }
    assert(rejected);
}
void UnconfiguredHardware() {
    const auto config = Config();
    const auto layout = ec::BuildPdoLayout(config);
    const auto& [name, capacity] = *config.adapters.begin();
    const auto& image = layout.adapters.at(name);
    std::vector<ec::BoardImage> boards{
        {name, capacity, image, std::vector<uint8_t>(image.rx.bytes),
         std::vector<uint8_t>(image.tx.bytes), false, false}};
    auto transport = ec::CreateIghTransport();
    std::string error;
    assert(!transport->Open(0, boards, error));
#if DUST_TEST_IGH
    assert(error.find("nonzero vendor_id") != std::string::npos);
#else
    assert(error.find("IgH support is not built") != std::string::npos);
#endif
    transport->Close();
}
void Application(const char* directory) {
    auto cfg = dust_control::app::LoadDustControlConfig(std::string(directory) +
                                                        "/dust_control.yaml");
    cfg.execution = {true, false, false};
    cfg.chassis.thread = cfg.gimbal.thread = cfg.ui.thread = {false, -1, 0};
    dust_control::app::DustControl app(cfg, directory);
    app.Start();
    std::this_thread::sleep_for(8ms);
    assert(!app.finished());
    app.Stop();
    assert(app.finished());
    dust_control::chassis::ChassisObservation sample{};
    uint64_t observations = 0, previous_ns = 0;
    while (app.chassis().TakeObservation(sample)) {
        assert(sample.sequence == ++observations &&
               sample.monotonic_ns > previous_ns);
        assert(sample.simulated);
        previous_ns = sample.monotonic_ns;
    }
    assert(observations > 0);
}
int main(int argc, char** argv) {
    assert(argc == 2);
    Commands();
    UnconfiguredHardware();
    MasterLifecycle(1, 0);
    MasterLifecycle(3, 0);
    for (unsigned fault = 1; fault <= 5; ++fault) {
        MasterLifecycle(3, fault);
    }
    Application(argv[1]);
}
