/* SPDX-License-Identifier: Apache-2.0 */
#include <algorithm>
#include <atomic>
#include <bit>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "../support/motor_fixture.hpp"
#include "chassis/runtime/realtime_loop.hpp"
#include "dust_control.hpp"
#include "hardware/ethercat/master.hpp"
#include "hardware/ethercat/wire.hpp"
namespace ec = dust_control::ethercat;
namespace host = dust_control::host;
namespace rt = dust_control::realtime;
namespace w = ec::wire;
using namespace std::chrono_literals;
uint32_t Offset(const ec::ProcessImageLayout& layout, const std::string& name) {
    for (const auto& m : layout.mappings) {
        for (const auto& e : m.entries) {
            if (e.name == name) {
                return e.offset;
            }
        }
    }
    throw std::runtime_error(name);
}
ec::PdoConfig Config(unsigned count = 1) {
    ec::PdoConfig config;
    for (unsigned i = 0; i < count; ++i) {
        ec::AdapterPdoConfig board;
        board.position = i;
        board.can_tx_slots = {6, 4};
        board.can_rx_slots = {0, 0, 4, 4};
        board.uart[0] = {16, 32};
        board.uart[1] = {128, 128};
        config.adapters.emplace("board" + std::to_string(i), board);
    }
    return config;
}
host::EthercatDeviceOptions NoSensors() {
    host::EthercatDeviceOptions options;
    options.uart_sensors = {0, 0, 0};
    options.decode_motors = false;
    return options;
}
void Header(std::vector<uint8_t>& image, const ec::AdapterPdoLayout& layout,
            uint32_t sequence, uint64_t nonce = 77, uint64_t boot = 99) {
    auto* p = image.data();
    (void)layout;
    w::Put32(p, uint32_t(boot));
    w::Put32(p + 4, uint32_t(nonce));
    w::Put32(p + 8, sequence);
    w::Put32(p + 12, uint32_t(rt::MonotonicTimeNs() / 1000));
    w::Put32(p + 16, w::kFirmwareProtection | 0x000f00ffU);
    p[20] = uint8_t(w::kActive);
}
void Commands() {
    auto cfg = Config();
    host::EthercatTransmitter tx(cfg.adapters.begin()->second);
    const std::array<host::CanCommandGroup, 1> groups{{{2, 0x200}}};
    assert(tx.ConfigureCan(host::CanProducer::kChassis, groups, 20000000));
    assert(!tx.ConfigureCan(host::CanProducer::kGimbal, groups, 20000000));
    const std::array<host::CanCommandGroup, 1> other{{{3, 0x200}}};
    assert(tx.ConfigureCan(host::CanProducer::kGimbal, other, 10000000));
    const auto now = rt::MonotonicTimeNs();
    tx.Activate(now);
    const std::array<dust_control::devices::CanFrame, 1> frame{
        {{2, 0x200, {1, 2, 3, 4, 5, 6, 7, 8}}}};
    assert(tx.SubmitCan(host::CanProducer::kChassis, frame, 1, now));
    assert(!tx.SubmitCan(host::CanProducer::kChassis, frame, 1, now));
    assert(
        !tx.SubmitCan(host::CanProducer::kChassis, frame, 2, now + 100000000));
    host::EthercatTransmitter::Batch batch;
    assert(tx.ReadBatch(0, now + 1000000, batch));
    assert(batch.valid && batch.count == 1 && batch.remaining_us == 19000);
    assert(tx.ReadBatch(0, now + 2000000, batch));
    assert(batch.sequence == 1 && batch.remaining_us == 18000);
    assert(!tx.ReadBatch(1, now + 10000000,
                         batch));  // Other source cannot extend its life.
    assert(!tx.ReadBatch(0, now + 20000000, batch));
    assert(tx.SubmitCan(host::CanProducer::kChassis, {}, 2, now));
    assert(tx.ReadBatch(0, now, batch) && batch.valid && batch.count == 0);

    std::array<uint8_t, 512> bytes{};
    for (size_t i = 0; i < bytes.size(); ++i) {
        bytes[i] = uint8_t(i);
    }
    assert(tx.SubmitUart(2, bytes, now));
    assert(!tx.SubmitUart(3, bytes, now));
    host::EthercatTransmitter::UartBlock block;
    for (uint32_t i = 1; i <= 4; ++i) {
        assert(tx.NextUart(1, 0, false, now, block));
        assert(block.valid && block.sequence == i && block.size == 128);
        assert(block.bytes[1] == uint8_t((i - 1) * 128 + 1));
    }
    assert(tx.NextUart(1, 0, false, now + 1000, block) &&
           block.sequence == 1);  // Bounded replay.
    assert(tx.NextUart(1, 4, true, now + 2000, block) && !block.valid);
    assert(!tx.NextUart(1, 100, true, now + 3000,
                        block));  // No ACK of unsent data.
    assert(tx.SubmitUart(2, bytes, now));
    assert(tx.NextUart(1, 4, true, now, block));
    assert(!tx.NextUart(1, 4, true, now + 20000000, block));  // ACK timeout.
    tx.RequestStop();
    assert(!tx.healthy());
}
void SessionCodec() {
    auto cfg = Config();
    const auto layout = ec::BuildPdoLayout(cfg).adapters.begin()->second;
    host::EthercatSession session(cfg.adapters.begin()->second, layout,
                                  NoSensors());
    std::vector<uint8_t> input(layout.tx.bytes), output(layout.rx.bytes);
    Header(input, layout, 1);
    const auto uart = Offset(layout.tx, "uart1.block_sequence");
    auto* p = input.data() + uart;
    w::Put32(p, 1);
    w::Put32(p + 8, w::U32(input.data() + 12));
    w::Put16(p + 12, 3);
    p[14] = w::kDataValid;
    p[15] = 4;
    p[16] = 5;
    p[17] = 6;
    const auto imu = Offset(layout.tx, "imu.sample_sequence");
    w::Put32(input.data() + imu, 1);
    w::Put32(input.data() + imu + 4, w::U32(input.data() + 12));
    input.data()[imu + 8] = 1;
    w::Put32(input.data() + imu + 9, std::bit_cast<uint32_t>(25.0f));
    const auto now = rt::MonotonicTimeNs();
    assert(session.Receive(input, now, 77));
    host::UartReceiveChunk chunk;
    assert(session.uart_queue().TakeUartChunk(1, chunk) && chunk.size == 3 &&
           chunk.bytes[2] == 6);
    const auto sample = session.onboard_imu().Read();
    assert(sample.sequence == 1 && sample.temperature_c == 25.0f);
    assert(session.Receive(input, now + 1000000, 77));
    assert(!session.uart_queue().TakeUartChunk(1, chunk));
    assert(session.onboard_imu().Read().sample_us == sample.sample_us);
    session.Activate(now);
    assert(session.Encode(output, now + 1000000, 77, 2, w::kArm));
    const auto uart_out = Offset(layout.rx, "uart1.block_sequence");
    assert(w::U32(output.data() + uart_out + 4) == 1);
    assert(output.data()[uart_out + 14] & w::kAckValid);
    const auto request = session.RequestStatus(1, 2, 3);
    assert(session.Encode(output, now + 1000000, 77, 3, w::kArm));
    assert(w::U16(output.data() + 16) == request && output[14] == 13 &&
           output[15] == 2);
    Header(input, layout, 2);
    w::Put32(p, 3);  // Missing UART block 2: no delivery or ACK advance.
    assert(session.Receive(input, now + 2000000, 77));
    assert(!session.uart_queue().TakeUartChunk(1, chunk));
    Header(input, layout, 3);
    w::Put32(p, 2);
    assert(session.Receive(input, now + 3000000, 77));
    assert(session.uart_queue().TakeUartChunk(1, chunk) && chunk.sequence == 2);
    assert(session.Encode(output, now + 3000000, 77, 4, w::kStop));
    assert(output[12] == w::kStop);
    assert(std::all_of(output.begin() + 18, output.end(),
                       [](uint8_t x) { return x == 0; }));
    assert(!session.Receive(input, now + 30000000, 77));

    for (unsigned fault = 0; fault < 5; ++fault) {
        host::EthercatSession bad(cfg.adapters.begin()->second, layout,
                                  NoSensors());
        std::fill(input.begin(), input.end(), 0);
        Header(input, layout, 1);
        if (fault == 0) {
            w::Put32(input.data(), 0);
        }
        if (fault == 1) {
            w::Put32(input.data() + 4, 123);
        }
        if (fault == 2) {
            w::Put32(input.data() + 16, 0);
        }
        if (fault == 3) {
            w::Put16(input.data() + uart + 12, 33);
        }
        if (fault == 4) {
            const auto can = Offset(layout.tx, "can2.batch_sequence");
            input.data()[can + 8] = 5;
        }
        assert(!bad.Receive(input, now, 77));
        assert(!bad.error().empty());
    }
    host::EthercatSession invalid_flags(cfg.adapters.begin()->second, layout, NoSensors());
    assert(!invalid_flags.Encode(output, now, 77, 1, 0x101));
    host::EthercatSession reboot(cfg.adapters.begin()->second, layout,
                                 NoSensors());
    std::fill(input.begin(), input.end(), 0);
    Header(input, layout, 1);
    assert(reboot.Receive(input, now, 77));
    Header(input, layout, 2, 77, 100);
    assert(!reboot.Receive(input, now, 77));
}
void CanSnapshotGap() {
    const auto cfg = Config();
    const auto layout = ec::BuildPdoLayout(cfg).adapters.begin()->second;
    auto options = NoSensors();
    options.decode_motors = true;
    host::EthercatSession session(cfg.adapters.begin()->second, layout, options,
                                  dust_control::test::MotorFixture());
    std::vector<uint8_t> input(layout.tx.bytes);
    Header(input, layout, 1);
    for (unsigned bus : {2U, 3U}) {
        auto* header =
            input.data() +
            Offset(layout.tx, "can" + std::to_string(bus) + ".batch_sequence");
        w::Put32(header, 1);
        header[8] = 1;
        auto* frame = header + 9;
        w::Put16(frame, 0x201);
        w::Put32(frame + 2, 1);
        w::Put32(frame + 6, w::U32(input.data() + 12));
        frame[10] = 8;
    }
    auto now = rt::MonotonicTimeNs();
    assert(session.Receive(input, now, 77));
    assert(session.motor_decoder().Read()[0].valid &&
           session.motor_decoder().Read()[1].valid);
    Header(input, layout, 2);
    auto* header = input.data() + Offset(layout.tx, "can3.batch_sequence");
    w::Put32(header, 2);
    w::Put32(header + 11, 3);  // Latest-value sequence may skip updates.
    now = rt::MonotonicTimeNs();
    assert(session.Receive(input, now, 77));
    assert(session.motor_decoder().Read()[0].valid);
    assert(session.motor_decoder().Read()[1].valid); // Coalescing is not a drop.
    const auto sample = session.motor_decoder().Read()[0].precise_timestamp_us;
    Header(input, layout, 3);
    assert(session.Receive(input, now + 1000000, 77));
    assert(session.motor_decoder().Read()[0].precise_timestamp_us == sample);
    Header(input, layout, 4);
    w::Put32(header, 3);
    w::Put32(header + 4, 1); // A real transport drop still resets unwrap state.
    w::Put32(header + 11, 4);
    assert(session.Receive(input, now + 2000000, 77));
    assert(!session.motor_decoder().Read()[1].valid);
}
void CanLatestSlots() {
    auto cfg = Config();
    const auto layout = ec::BuildPdoLayout(cfg).adapters.begin()->second;
    host::EthercatSession session(cfg.adapters.begin()->second, layout, NoSensors());
    std::vector<uint8_t> input(layout.tx.bytes);
    auto *h = input.data() + Offset(layout.tx, "can3.batch_sequence");
    Header(input, layout, 1);
    w::Put32(h, 1); h[8] = 2;
    auto *a = h + 9; auto *b = a + 19;
    w::Put16(a, 0x201); w::Put32(a + 2, 100); a[10] = 8;
    w::Put16(b, 0x013); w::Put32(b + 2, 99); b[10] = 8;
    auto now = rt::MonotonicTimeNs();
    assert(session.Receive(input, now, 77));
    Header(input, layout, 2); w::Put32(h, 2);
    w::Put32(a + 2, 103); // Other ID can remain unchanged and older.
    assert(session.Receive(input, now + 1000000, 77));
    Header(input, layout, 3); w::Put32(h, 3);
    b[11] = 1; // Reusing an ID sequence with different bytes is invalid.
    assert(!session.Receive(input, now + 2000000, 77));
}
void UnconfiguredHardware() {
    auto config = Config();
    auto layout = ec::BuildPdoLayout(config);
    const auto& adapter = *config.adapters.begin();
    const auto& image = layout.adapters.at(adapter.first);
    std::vector<ec::BoardImage> boards{{adapter.first, adapter.second, image,
                                        std::vector<uint8_t>(image.rx.bytes),
                                        std::vector<uint8_t>(image.tx.bytes),
                                        false, false}};
    auto transport = ec::CreateIghTransport();
    std::string error;
    // Both real and driver-free builds reject this before opening a master.
    assert(!transport->Open(0, boards, error));
#if DUST_TEST_IGH
    assert(error.find("nonzero vendor_id") != std::string::npos);
#else
    assert(error.find("IgH support is not built") != std::string::npos);
#endif
    transport->Close();
}
struct Observation {
    std::atomic<unsigned> stops{0}, sends{0}, receives{0};
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
        if (fault_ == 5 && cycle == 8) {
            std::this_thread::sleep_for(22ms);
        }
        for (auto& board : boards) {
            board.operational = true;
            board.complete = !(fault_ == 1 && cycle > 8);
            const auto seq = fault_ == 3 && cycle > 8 ? 8 : cycle;
            Header(board.input, board.layout, seq, nonce_,
                   fault_ == 2 && cycle > 8 ? 101 : 99);
            if (!armed_) {
                board.input[20] = uint8_t(w::kReady);
                w::Put32(board.input.data() + 4, 0);
            }
            if (fault_ == 4) {
                w::Put32(board.input.data() + 16, 0);
            }
        }
        return true;
    }
    bool Send(const std::vector<ec::BoardImage>& boards,
              std::string&) override {
        CheckThread();
        ++observation_->sends;
        bool all_stop = true;
        for (const auto& board : boards) {
            nonce_ = w::U32(board.output.data());
            assert(nonce_ != 0);
            const auto flags = board.output[12];
            armed_ = armed_ || flags == w::kArm;
            all_stop = all_stop && flags == w::kStop;
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
    uint64_t nonce_ = 0;
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
        auto session = std::make_unique<host::EthercatSession>(
            capacity, layout.adapters.at(name), NoSensors());
        master.Attach(name, *session);
        sessions.emplace(name, std::move(session));
    }
    const bool started = master.Start();
    if (fault == 4) {
        assert(!started && !master.error().empty());
    } else {
        assert(started && master.healthy());
        if (fault) {
            const auto deadline = std::chrono::steady_clock::now() + 1s;
            while (master.healthy() &&
                   std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(1ms);
            }
            assert(!master.healthy() && !master.error().empty());
        } else {
            std::this_thread::sleep_for(5ms);
        }
    }
    master.Stop();
    assert(observation->closed && !observation->wrong_thread &&
           observation->stops == 25);
    for (const auto& [name, session] : sessions) {
        (void)name;
        assert(!session->healthy());
    }
    assert(master.statistics().cycles > 0);
    bool rejected = false;
    try {
        master.Start();
    } catch (const std::logic_error&) {
        rejected = true;
    }
    assert(rejected);
}
void Application(const char* directory) {
    auto cfg = dust_control::app::LoadDustControlConfig(std::string(directory) +
                                                        "/dust_control.yaml");
    cfg.execution = {true, false, false};
    cfg.chassis.thread = {false, -1, 0};
    cfg.gimbal.thread = cfg.ui.thread = {false, -1, 0};
    dust_control::app::DustControl app(cfg, directory);
    app.Start();
    std::this_thread::sleep_for(8ms);
    assert(!app.finished());
    app.Stop();
    assert(app.finished());
    cfg.execution.enable_output = true;
    bool rejected = false;
    try {
        dust_control::app::ValidateDustControlConfig(cfg);
    } catch (const std::exception&) {
        rejected = true;
    }
    assert(rejected);
}
void FastCycleCodec() {
    for (auto period : {125U, 250U, 500U, 1000U}) {
        auto config = Config();
        config.cycle_time_us = period;
        auto& capacity = config.adapters.at("board0");
        capacity.can_tx_slots = {};
        capacity.can_rx_slots = {0, 0, 0, 1};
        capacity.uart = {};
        const auto layout = ec::BuildPdoLayout(config).adapters.at("board0");
        host::EthercatSession session(capacity, layout, NoSensors());
        std::vector<uint8_t> output(layout.rx.bytes);
        assert(
            session.Encode(output, rt::MonotonicTimeNs(), 77, 1, w::kDiscover));
        assert(layout.cycle_time_us == period && w::U32(output.data()) == 77);
    }
}
int main(int argc, char** argv) {
    FastCycleCodec();
    assert(argc == 2);
    Commands();
    SessionCodec();
    CanSnapshotGap();
    CanLatestSlots();
    UnconfiguredHardware();
    MasterLifecycle(1, 0);
    MasterLifecycle(3, 0);
    for (unsigned fault = 1; fault <= 5; ++fault) {
        MasterLifecycle(3, fault);
    }
    Application(argv[1]);
    std::cout << "EtherCAT command deadlines, codec, UART windows, master "
                 "faults and app lifecycle passed\n";
}
