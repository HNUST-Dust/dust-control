/* SPDX-License-Identifier: Apache-2.0 */
// DM 失能帧台架：单个在途请求、空闲探测、主机同一单调时钟测量。
#include <ecrt.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/prctl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "hardware/ethercat/pdo_config.hpp"
#include "hardware/ethercat/wire.hpp"
namespace {
using namespace dust_control::ethercat;
struct Copy {
    unsigned image, domain, bytes;
};
void Check(bool value, const char* message) {
    if (!value) {
        throw std::runtime_error(message);
    }
}
void Map(ec_slave_config_t* sc, ec_domain_t* domain,
         const ProcessImageLayout& image, bool output,
         std::vector<Copy>& copies) {
    std::vector<std::vector<ec_pdo_entry_info_t>> entries(
        image.mappings.size());
    std::vector<ec_pdo_info_t> pdos;
    for (size_t i = 0; i < image.mappings.size(); ++i) {
        for (const auto& e : image.mappings[i].entries) {
            entries[i].push_back({e.object_index, e.subindex, e.bit_length});
        }
        pdos.push_back({image.mappings[i].index, unsigned(entries[i].size()),
                        entries[i].data()});
    }
    ec_sync_info_t sync[] = {
        {uint8_t(output ? 2 : 3), output ? EC_DIR_OUTPUT : EC_DIR_INPUT,
         unsigned(pdos.size()), pdos.data(),
         output ? EC_WD_ENABLE : EC_WD_DISABLE},
        {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DEFAULT}};
    Check(ecrt_slave_config_pdos(sc, EC_END, sync) == 0, "PDO mapping failed");
    for (const auto& mapping : image.mappings) {
        for (const auto& e : mapping.entries) {
            unsigned bit = 0;
            int offset = ecrt_slave_config_reg_pdo_entry(
                sc, e.object_index, e.subindex, domain, &bit);
            Check(offset >= 0 && !bit, "PDO registration failed/alignment");
            copies.push_back(
                {e.offset, unsigned(offset), unsigned(e.bit_length / 8)});
        }
    }
}
struct Master {
    ec_master_t* ptr{};
    ~Master() {
        if (ptr) {
            ecrt_release_master(ptr);
        }
    }
};
}  // namespace

using Clock = std::chrono::steady_clock;
static int64_t Ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               Clock::now().time_since_epoch())
        .count();
}
struct Sample {
    int64_t tx, rx;
    double wake;
};
static void Report(const std::string& path,
                   const std::vector<Sample>& samples) {
    std::ofstream csv(path);
    Check(bool(csv), "Cannot write CSV");
    csv << "sample,tx_ns,rx_ns,rtt_us,wake_lateness_us\n"
        << std::fixed << std::setprecision(3);
    std::vector<double> v;
    double sum = 0, squared = 0, delta = 0;
    for (size_t i = 0; i < samples.size(); ++i) {
        const auto& s = samples[i];
        const double us = (s.rx - s.tx) / 1000.0;
        csv << i << ',' << s.tx << ',' << s.rx << ',' << us << ',' << s.wake
            << '\n';
        v.push_back(us);
        sum += us;
        if (i) {
            delta = std::max(
                delta, std::abs(us - (samples[i - 1].rx - samples[i - 1].tx) /
                                         1000.0));
        }
    }
    Check(bool(csv), "CSV write failed");
    if (v.empty()) {
        std::cout << "No RTT samples.\n";
        return;
    }
    const double mean = sum / v.size();
    for (auto x : v) {
        squared += (x - mean) * (x - mean);
    }
    std::sort(v.begin(), v.end());
    auto p = [&](double f) { return v[size_t(std::ceil(f * v.size())) - 1]; };
    std::cout << std::fixed << std::setprecision(3) << "samples=" << v.size()
              << " RTT_us min=" << v.front() << " mean=" << mean
              << " p50=" << p(.5) << " p95=" << p(.95) << " p99=" << p(.99)
              << " max=" << v.back()
              << " stddev=" << std::sqrt(squared / v.size())
              << " peak_to_peak=" << v.back() - v.front()
              << " max_adjacent_change=" << delta << std::endl;
}
int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cerr << "Usage: dust_ethercat_dm_rtt CONFIG.yaml RESULTS.csv "
                     "[samples=1000]\nOnly CAN3 ID3 disable FF..FD; no motor "
                     "enable.\n";
        return 2;
    }
    std::vector<Sample> samples;
    try {
        const unsigned target = argc == 4 ? std::stoul(argv[3]) : 1000;
        Check(target && target <= 2000, "Sample count must be 1..2000");
        samples.reserve(target);
        std::ofstream preflight(argv[2]);
        Check(bool(preflight), "Cannot create CSV");
        preflight.close();
        prctl(PR_SET_TIMERSLACK, 1UL);
        cpu_set_t allowed;
        CPU_ZERO(&allowed);
        if (!sched_getaffinity(0, sizeof(allowed), &allowed)) {
            for (int cpu = CPU_SETSIZE - 1; cpu >= 0; --cpu) {
                if (CPU_ISSET(cpu, &allowed)) {
                    cpu_set_t one;
                    CPU_ZERO(&one);
                    CPU_SET(cpu, &one);
                    Check(!sched_setaffinity(0, sizeof(one), &one),
                          "Cannot set affinity");
                    std::cout << "CPU=" << cpu << std::endl;
                    break;
                }
            }
        }
        sched_param priority{};
        priority.sched_priority = 40;
        const bool realtime = sched_setscheduler(0, SCHED_FIFO, &priority) == 0;
        std::cout << "scheduler="
                  << (realtime ? "FIFO40" : "OTHER (no realtime privilege)")
                  << " wait=adaptive (RT:40us tail; OTHER:150us "
                     "tail/full-spin<=250us) mlock="
                  << (mlockall(MCL_CURRENT | MCL_FUTURE) == 0) << std::endl;
        const auto config = LoadPdoConfig(argv[1]);
        Check(config.adapters.size() == 1, "Requires exactly one lab slave");
        const auto& c = config.adapters.begin()->second;
        Check(c.vendor_id == 0 && c.product_code == 0x44544331 && !c.alias &&
                  !c.position,
              "Requires lab VID=0 PID=0x44544331 alias=0 position=0");
        Check(c.can_tx_slots[0] == 1 && !c.can_tx_slots[1] && !c.onboard_imu,
              "Requires exactly one chassis TX slot and no IMU");
        Check(c.can_rx_slots[3] >= 1 && !c.can_rx_slots[0] &&
                  !c.can_rx_slots[1] && !c.can_rx_slots[2],
              "Requires only CAN3 feedback");
        for (const auto& uart : c.uart) {
            Check(!uart.tx_bytes && !uart.rx_bytes, "No UART traffic allowed");
        }
        const auto all = BuildPdoLayout(config);
        const auto& layout = all.adapters.begin()->second;
        Master master{ecrt_request_master(config.master_index)};
        Check(master.ptr, "Cannot request IgH master");
        Check(ecrt_master_set_send_interval(master.ptr, config.cycle_time_us) ==
                  0,
              "Cannot set IgH send interval");
        ec_master_info_t info{};
        Check(ecrt_master(master.ptr, &info) == 0 && info.slave_count == 1,
              "Requires exactly one connected slave");
        ec_slave_info_t identity{};
        Check(ecrt_master_get_slave(master.ptr, 0, &identity) == 0 &&
                  identity.vendor_id == 0 &&
                  identity.product_code == 0x44544331 &&
                  identity.revision_number == 0x10000,
              "Live identity mismatch; update gateway firmware first");
        auto* domain = ecrt_master_create_domain(master.ptr);
        Check(domain, "Cannot create domain");
        auto* sc = ecrt_master_slave_config(master.ptr, c.alias, c.position,
                                            c.vendor_id, c.product_code);
        Check(sc, "Cannot configure slave");
        const uint32_t values[] = {layout.layout_signature,
                                   config.cycle_time_us,
                                   20000,
                                   c.can_tx_slots[0],
                                   c.can_tx_slots[1],
                                   c.can_rx_slots[0],
                                   c.can_rx_slots[1],
                                   c.can_rx_slots[2],
                                   c.can_rx_slots[3],
                                   c.uart[0].tx_bytes,
                                   c.uart[0].rx_bytes,
                                   c.uart[1].tx_bytes,
                                   c.uart[1].rx_bytes,
                                   c.uart[2].tx_bytes,
                                   c.uart[2].rx_bytes,
                                   c.uart[3].tx_bytes,
                                   c.uart[3].rx_bytes,
                                   c.uart[4].tx_bytes,
                                   c.uart[4].rx_bytes,
                                   c.uart[5].tx_bytes,
                                   c.uart[5].rx_bytes,
                                   0};
        for (unsigned i = 0; i < 22; ++i) {
            Check(ecrt_slave_config_sdo32(sc, 0x2100, i + 1, values[i]) == 0,
                  "Cannot queue configuration SDO");
        }
        Check(ecrt_slave_config_sdo16(sc, 0x1c32, 1, 0) == 0 &&
                  ecrt_slave_config_sdo16(sc, 0x1c33, 1, 0) == 0,
              "Cannot queue FreeRun synchronization selection");
        std::vector<Copy> output, input;
        Map(sc, domain, layout.rx, true, output);
        Map(sc, domain, layout.tx, false, input);
        Check(ecrt_master_activate(master.ptr) == 0, "Cannot activate master");
        auto* data = ecrt_domain_data(domain);
        Check(data, "Cannot access domain");

        auto offset = [](const ProcessImageLayout& image,
                         const std::string& name) {
            for (const auto& m : image.mappings) {
                for (const auto& e : m.entries) {
                    if (e.name == name) {
                        return e.offset;
                    }
                }
            }
            throw std::runtime_error("Missing field: " + name);
        };
        std::vector<uint8_t> rx(layout.rx.bytes), tx(layout.tx.bytes);
        const auto source = offset(layout.rx, "can.chassis.source_sequence");
        const auto feedback = offset(layout.tx, "can3.batch_sequence");
        const uint64_t session = uint64_t(Ns()) | 1;
        wire::Put32(rx.data(), wire::kMagic);
        wire::Put16(rx.data() + 4, wire::kMajor);
        wire::Put64(rx.data() + 8, session);
        wire::Put32(rx.data() + 28, wire::kDiscover);
        wire::Put32(rx.data() + 32, layout.layout_signature);
        wire::Put32(rx.data() + 40, config.cycle_time_us);
        uint64_t boot = 0;
        uint32_t batch = 0, last_feedback = 0;
        bool have_feedback = false, armed = false, pending = false,
             new_command = false;
        int64_t quiet_start = 0, sent = 0, next_send = 0, last_pdo_send = 0;
        double pending_wake = 0;
        unsigned wkc_misses = 0, commands = 0;
        auto next = Clock::now();
        const auto deadline = next + std::chrono::seconds(35);
        std::cout << "PDO Rx=" << rx.size() << " Tx=" << tx.size()
                  << " signature=" << layout.layout_signature
                  << " cycle_us=" << config.cycle_time_us
                  << " command_interval_us=10000 timeout_us=8000" << std::endl;
        for (uint32_t exchange = 1; Clock::now() < deadline; ++exchange) {
            const auto woke = Clock::now();
            const auto wake =
                std::chrono::duration<double, std::micro>(woke - next).count();
            next += std::chrono::microseconds(config.cycle_time_us);
            ecrt_master_receive(master.ptr);
            const int64_t received = Ns();
            ecrt_domain_process(domain);
            ec_domain_state_t ds{};
            ec_slave_config_state_t ss{};
            ecrt_domain_state(domain, &ds);
            ecrt_slave_config_state(sc, &ss);
            bool active = false;
            if (ss.operational && ds.wc_state == EC_WC_COMPLETE) {
                for (const auto& e : input) {
                    std::memcpy(tx.data() + e.image, data + e.domain, e.bytes);
                }
                if (wire::U32(tx.data()) == wire::kMagic) {
                    Check(wire::U16(tx.data() + 4) == wire::kMajor &&
                              wire::U32(tx.data() + 32) ==
                                  layout.layout_signature,
                          "ABI/layout mismatch");
                    const auto state = wire::U32(tx.data() + 40),
                               fault = wire::U32(tx.data() + 44);
                    Check(!fault && (state == wire::kReady ||
                                     (armed && state == wire::kActive)),
                          ("Slave state=" + std::to_string(state) +
                           " fault=" + std::to_string(fault))
                              .c_str());
                    Check(wire::U32(tx.data() + 36) & wire::kFirmwareProtection,
                          "No firmware protection");
                    const auto b = wire::U64(tx.data() + 8);
                    Check(b && (!boot || boot == b), "Boot changed");
                    boot = b;
                    active = state == wire::kActive;
                    Check(active ? wire::U64(tx.data() + 16) == session
                                 : !wire::U64(tx.data() + 16),
                          "Session mismatch");
                    const auto* f = tx.data() + feedback;
                    const auto seq = wire::U32(f);
                    const auto count = wire::U16(f + 8);
                    Check(count <= c.can_rx_slots[3], "Invalid feedback count");
                    if (!have_feedback || wire::Newer(seq, last_feedback)) {
                        have_feedback = true;
                        last_feedback = seq;
                        for (unsigned n = 0; n < count; ++n) {
                            const auto* frame = f + 16 + 24 * n;
                            if (wire::U32(frame) != 0x13) {
                                continue;
                            }
                            Check(frame[12] == 8 && (frame[16] & 15) == 3,
                                  "Unexpected DM frame");
                            if (!armed && quiet_start &&
                                received - quiet_start < 100000000) {
                                continue;
                            }
                            Check(pending,
                                  "Unsolicited DM feedback; cannot pair RTT");
                            Check(wire::U32(tx.data() + 48) == batch,
                                  "Feedback without accepted command");
                            Check((frame[16] >> 4) == 0,
                                  "DM did not report disabled state");
                            Check(received > sent && received - sent <= 8000000,
                                  "Response outside 8ms window");
                            samples.push_back({sent, received, pending_wake});
                            pending = false;
                        }
                    }
                    if (!armed) {
                        if (!quiet_start) {
                            quiet_start = received;
                        }
                        if (received - quiet_start >= 1000000000 &&
                            wire::U32(tx.data() + 56) == 1) {
                            armed = true;
                            wire::Put64(rx.data() + 16, boot);
                            wire::Put32(rx.data() + 28, wire::kArm);
                            wire::Put32(rx.data() + 36, 1);
                        }
                    }
                }
            } else if (armed) {
                ++wkc_misses;
                // 仅诊断迟到帧：即使稍后收到仍判失败，不把周期违约隐藏为通过。
                const auto initial_wc = ds.working_counter;
                const auto check_started = Ns();
                while (ds.wc_state != EC_WC_COMPLETE &&
                       Ns() - check_started < 1000000) {
                    ecrt_master_receive(master.ptr);
                    ecrt_domain_process(domain);
                    ecrt_domain_state(domain, &ds);
                }
                const auto last_receive = Ns();
                std::cerr << "RX deadline diagnostic: initial_wc=" << initial_wc
                          << " eventual_wc=" << ds.working_counter
                          << " first_poll_after_send_us="
                          << (check_started - last_pdo_send) / 1000.0
                          << " final_poll_after_send_us="
                          << (last_receive - last_pdo_send) / 1000.0 << std::endl;
                throw std::runtime_error(
                    ("OP/WKC lost during measurement: AL=" +
                     std::to_string(ss.al_state) +
                     " WC=" + std::to_string(ds.working_counter) +
                     " wc_state=" + std::to_string(ds.wc_state))
                        .c_str());
            }
            if (pending && Ns() - sent > 8000000) {
                throw std::runtime_error(
                    "DM feedback timeout (no retry to avoid late-response "
                    "pairing)");
            }
            if (samples.size() == target) {
                std::fill(rx.begin() + 64, rx.end(), 0);
                wire::Put32(rx.data() + 28, wire::kStop);
                wire::Put32(rx.data() + 24, exchange);
                for (const auto& e : output) {
                    std::memcpy(data + e.domain, rx.data() + e.image, e.bytes);
                }
                ecrt_domain_queue(domain);
                ecrt_master_send(master.ptr);
                Report(argv[2], samples);
                std::cout << "PASS commands=" << commands
                          << " timeouts=0 WKC_misses=" << wkc_misses
                          << "; sequential DM pairing, no sequence echo; no "
                             "hard-RT claim.\n";
                return 0;
            }
            new_command = false;
            if (active && !pending && Ns() >= next_send) {
                auto* p = rx.data() + source;
                wire::Put32(p, ++batch);
                wire::Put32(p + 4, 20000);
                wire::Put16(p + 8, 1);
                p[10] = 1;
                p[11] = 0;
                p[16] = 3;
                p[17] = 8;
                wire::Put32(p + 20, 3);
                std::fill(p + 24, p + 31, 0xff);
                p[31] = 0xfd;
                new_command = true;
                pending_wake = wake;
            }
            wire::Put32(rx.data() + 24, exchange);
            wire::Put32(rx.data() + 44, 1);
            for (const auto& e : output) {
                std::memcpy(data + e.domain, rx.data() + e.image, e.bytes);
            }
            ecrt_domain_queue(domain);
            last_pdo_send = Ns();
            if (new_command) {
                sent = last_pdo_send;
                next_send = sent + 10000000;
                pending = true;
                ++commands;
            }
            ecrt_master_send(master.ptr);
            if (Clock::now() > next + std::chrono::milliseconds(5)) {
                next = Clock::now();
            }
            // 普通调度下先提前唤醒，末段忙等，避免把系统定时器唤醒误差当作链路耗时。
            const auto spin_begin =
                next - std::chrono::microseconds(realtime ? 40 : 150);
            if ((realtime || config.cycle_time_us > 250) &&
                Clock::now() < spin_begin) {
                std::this_thread::sleep_until(spin_begin);
            }
            while (Clock::now() < next) {
                // 保持可抢占；不改变调度权限，也不丢弃迟到样本。
#if defined(__x86_64__) || defined(__i386__)
                asm volatile("pause");
#else
                std::atomic_signal_fence(std::memory_order_seq_cst);
#endif
            }
        }
        throw std::runtime_error("Benchmark deadline exceeded");
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << std::endl;
        try {
            Report(argv[2], samples);
        } catch (const std::exception& report) {
            std::cerr << report.what() << std::endl;
        }
        return 1;
    }
}
