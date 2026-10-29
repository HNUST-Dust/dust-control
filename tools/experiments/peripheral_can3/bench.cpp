/* SPDX-License-Identifier: Apache-2.0 */
// CAN3 三电机失能/零力矩台架；单个在途 DM 请求，UART2/5 可靠 ACK。
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
    uint64_t boot = 0, session = 0;
    uint32_t batch = 0, publication = 0, can_sequence = 0, exchange = 0;
    int64_t built = 0, encoded = 0, output_copied = 0, queued = 0,
            send_enter = 0, send_return = 0, receive_enter = 0;
    int64_t processed = 0, copied = 0, decoded = 0, consumed = 0, ack = 0;
};
struct UartChunk {
    int64_t ns = 0;
    uint32_t sequence = 0;
    uint16_t length = 0, flags = 0;
    unsigned port = 0;
    std::array<uint8_t, 128> bytes{};
};
struct CanSnapshot {
    int64_t ns = 0;
    uint32_t exchange=0, publication=0, accepted=0, batch=0;
    uint16_t count=0;
    std::array<uint32_t,32> ids{}, sequences{};
};
static void ReportCan(const std::string& path,
                      const std::array<CanSnapshot,64>& ring, unsigned written) {
    std::ofstream out(path + ".can.csv");
    Check(bool(out), "Cannot write CAN snapshot history");
    out << "receive_ns,host_exchange,publication,accepted,can_batch,count,ids,sequences\n";
    const auto first=written>ring.size()?written-ring.size():0;
    for(unsigned i=first;i<written;++i) {
        const auto& s=ring[i%ring.size()];
        out << s.ns << ',' << s.exchange << ',' << s.publication << ','
            << s.accepted << ',' << s.batch << ',' << s.count << ',';
        for(unsigned n=0;n<s.count;++n) out << (n?";":"") << s.ids[n];
        out << ',';
        for(unsigned n=0;n<s.count;++n) out << (n?";":"") << s.sequences[n];
        out << '\n';
    }
    Check(bool(out), "CAN history write failed");
}
static void ReportUart(const std::string& path,
                       const std::vector<UartChunk>& chunks) {
    std::ofstream out(path + ".uart.csv");
    Check(bool(out), "Cannot write UART capture");
    out << "receive_ns,port,sequence,flags,data_hex\n";
    for (const auto& c : chunks) {
        out << std::dec << c.ns << ',' << c.port << ',' << c.sequence << ','
            << c.flags << ',';
        for (unsigned i = 0; i < c.length; ++i)
            out << std::hex << std::setw(2) << std::setfill('0')
                << unsigned(c.bytes[i]);
        out << '\n';
    }
    Check(bool(out), "UART capture write failed");
    for (unsigned port : {2U, 5U}) {
        size_t count = 0, bytes = 0, gaps = 0;
        for (const auto& c : chunks) if (c.port == port) {
            ++count; bytes += c.length; gaps += bool(c.flags & wire::kDiscontinuity);
        }
        std::cout << "UART" << port << " blocks=" << count << " bytes=" << bytes
                  << " discontinuities=" << gaps << std::endl;
    }
}
static void Report(const std::string& path,
                   const std::vector<Sample>& samples) {
    std::ofstream csv(path);
    Check(bool(csv), "Cannot write CSV");
    csv << "sample,tx_ns,rx_ns,rtt_us,wake_lateness_us,boot,session,batch,"
           "publication,can_sequence,exchange,built_ns,encoded_ns,output_"
           "copied_ns,queued_ns,send_enter_ns,send_return_ns,receive_enter_ns,"
           "domain_processed_ns,image_copied_ns,decoded_ns,bench_consumed_ns,"
           "accepted_observed_ns\n"
        << std::fixed << std::setprecision(3);
    std::vector<double> v;
    double sum = 0, squared = 0, delta = 0;
    for (size_t i = 0; i < samples.size(); ++i) {
        const auto& s = samples[i];
        const double us = (s.rx - s.tx) / 1000.0;
        csv << i << ',' << s.tx << ',' << s.rx << ',' << us << ',' << s.wake
            << ',' << s.boot << ',' << s.session << ',' << s.batch << ','
            << s.publication << ',' << s.can_sequence << ',' << s.exchange
            << ',' << s.built << ',' << s.encoded << ',' << s.output_copied
            << ',' << s.queued << ',' << s.send_enter << ',' << s.send_return
            << ',' << s.receive_enter << ',' << s.processed << ',' << s.copied
            << ',' << s.decoded << ',' << s.consumed << ',' << s.ack << '\n';
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
        std::cerr << "Usage: dust_ethercat_peripheral_can3_rtt CONFIG.yaml RESULTS.csv "
                     "[samples=1000]\nCAN3 ID0x200 zero torque, ID0/3 disable; "
                     "UART2/5 RX only; no motor enable.\n";
        return 2;
    }
    std::vector<Sample> samples;
    std::vector<UartChunk> uart_chunks;
    uart_chunks.reserve(40000);
    std::array<CanSnapshot,64> can_history{};
    unsigned can_history_count=0;
    try {
        const unsigned target = argc == 4 ? std::stoul(argv[3]) : 1000;
        Check(target && target <= 2000, "Sample count must be 1..2000");
#ifdef __OPTIMIZE__
        std::cout << "compiler_optimized=1" << std::endl;
#else
        std::cout << "compiler_optimized=0" << std::endl;
#endif
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
        Check(c.can_tx_slots[0] >= 6 && !c.can_tx_slots[1],
              "Requires >=6 chassis TX slots and no gimbal");
        Check(!c.can_rx_slots[2] && c.can_rx_slots[3] >= 1 &&
                  !c.can_rx_slots[0] && !c.can_rx_slots[1],
              "Requires CAN3 feedback only; CAN2 excluded");
        Check(!c.onboard_imu && c.uart[1].rx_bytes == 128 &&
                  c.uart[4].rx_bytes == 32, "Requires UART2/5 RX configuration");
        for (const auto& u : c.uart) Check(!u.tx_bytes, "UART TX forbidden");
        // UART and onboard IMU are allowed; bench uses RX-only UART sensors.
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
        const unsigned uart_in[] = {offset(layout.tx, "uart2.block_sequence"),
                                    offset(layout.tx, "uart5.block_sequence")};
        const unsigned uart_out[] = {offset(layout.rx, "uart2.block_sequence"),
                                     offset(layout.rx, "uart5.block_sequence")};
        uint32_t uart_sequences[2]{};
        uint32_t can_sequence = 0;
        bool have_can_sequence = false;
        unsigned can_counts[3]{};
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
        Sample trace{};
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
            const int64_t receive_enter = Ns();
            ecrt_master_receive(master.ptr);
            const int64_t received = Ns();
            ecrt_domain_process(domain);
            const int64_t processed = Ns();
            ec_domain_state_t ds{};
            ec_slave_config_state_t ss{};
            ecrt_domain_state(domain, &ds);
            ecrt_slave_config_state(sc, &ss);
            bool active = false;
            if ((exchange & 0x1FFF) == 0) {
                std::cerr << "DIAG al_state=" << ss.al_state
                          << " operational=" << ss.operational
                          << " wc=" << ds.working_counter
                          << " wc_state=" << ds.wc_state
                          << " exchange=" << exchange << std::endl;
            }
            if (ss.operational && ds.wc_state == EC_WC_COMPLETE) {
                for (const auto& e : input) {
                    std::memcpy(tx.data() + e.image, data + e.domain, e.bytes);
                }
                const int64_t copied = Ns();
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
                    if (active) for (unsigned u = 0; u < 2; ++u) {
                        const auto* p = tx.data() + uart_in[u];
                        const auto sequence = wire::U32(p);
                        const auto length = wire::U16(p + 12);
                        const auto flags = wire::U16(p + 14);
                        Check(length <= (u ? 32 : 128) && !(flags & ~7U),
                              "Invalid UART block");
                        if (!(flags & wire::kDataValid)) {
                            Check(!length, "UART length without valid flag");
                            continue;
                        }
                        if (!wire::Newer(sequence, uart_sequences[u])) continue;
                        if (!(flags & wire::kDiscontinuity) &&
                            sequence != uart_sequences[u] + 1) continue;
                        Check(uart_chunks.size() < uart_chunks.capacity(),
                              "UART capture capacity exceeded");
                        UartChunk chunk{};
                        chunk.ns = received; chunk.sequence = sequence;
                        chunk.length = length; chunk.flags = flags;
                        chunk.port = u ? 5 : 2;
                        std::copy_n(p + 16, length, chunk.bytes.begin());
                        uart_chunks.push_back(chunk);
                        uart_sequences[u] = sequence;
                        wire::Put32(rx.data() + uart_out[u] + 4, sequence);
                        wire::Put16(rx.data() + uart_out[u] + 14, wire::kAckValid);
                    }
                    Check(active ? wire::U64(tx.data() + 16) == session
                                 : !wire::U64(tx.data() + 16),
                          "Session mismatch");
                    if (pending && !trace.ack &&
                        wire::U32(tx.data() + 48) == batch) {
                        trace.ack = Ns();
                    }
                    const auto* f = tx.data() + feedback;
                    const auto seq = wire::U32(f);
                    const auto count = wire::U16(f + 8);
                    Check(count <= c.can_rx_slots[3], "Invalid feedback count");
                    if (!have_feedback || wire::Newer(seq, last_feedback)) {
                        have_feedback = true;
                        last_feedback = seq;
                        auto& observed=can_history[can_history_count++ % can_history.size()];
                        observed={}; observed.ns=received; observed.exchange=exchange;
                        observed.publication=wire::U32(tx.data()+24);
                        observed.accepted=wire::U32(tx.data()+48);
                        observed.batch=seq; observed.count=count;
                        Check(count<=observed.ids.size(), "CAN diagnostic capacity");
                        for(unsigned n=0;n<count;++n) {
                            observed.ids[n]=wire::U32(f+16+24*n);
                            observed.sequences[n]=wire::U32(f+20+24*n);
                        }
                        for (unsigned n = 0; n < count; ++n) {
                            const auto* frame = f + 16 + 24 * n;
                            const auto frame_sequence = wire::U32(frame + 4);
                            // Experimental windows can repeat an already consumed
                            // prefix; count and pair each receive sequence once.
                            if (have_can_sequence && !wire::Newer(frame_sequence, can_sequence))
                                continue;
                            if (!have_can_sequence || wire::Newer(frame_sequence, can_sequence)) {
                                have_can_sequence = true; can_sequence = frame_sequence;
                                const auto id = wire::U32(frame);
                                if(id == 0x201) ++can_counts[0];
                                if(id == 0x10) ++can_counts[1];
                                if(id == 0x13) ++can_counts[2];
                            }
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
                            trace.tx = sent;
                            trace.rx = received;
                            trace.wake = pending_wake;
                            trace.boot = boot;
                            trace.session = session;
                            trace.batch = batch;
                            trace.publication = wire::U32(tx.data() + 24);
                            trace.can_sequence = wire::U32(frame + 4);
                            trace.receive_enter = receive_enter;
                            trace.processed = processed;
                            trace.copied = copied;
                            trace.decoded = Ns();
                            trace.consumed =
                                Ns();  // Bench consumer; controller is not
                                       // running.
                            samples.push_back(trace);
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
                          << (last_receive - last_pdo_send) / 1000.0
                          << std::endl;
                throw std::runtime_error(
                    ("OP/WKC lost during measurement: AL=" +
                     std::to_string(ss.al_state) +
                     " WC=" + std::to_string(ds.working_counter) +
                     " wc_state=" + std::to_string(ds.wc_state))
                        .c_str());
            }
            if (pending && Ns() - sent > 8000000) {
                std::cerr << "TIMEOUT command_batch=" << batch
                          << " commands_sent=" << commands
                          << " ack_observed=" << bool(trace.ack)
                          << " last_accepted=" << wire::U32(tx.data()+48)
                          << " publication=" << wire::U32(tx.data()+24)
                          << " last_can_batch=" << last_feedback << std::endl;
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
                ReportUart(argv[2], uart_chunks);
                ReportCan(argv[2], can_history, can_history_count);
                std::cout << "CAN3 unique feedback 0x201=" << can_counts[0]
                          << " 0x10=" << can_counts[1] << " 0x13=" << can_counts[2] << std::endl;
                std::cout << "PASS commands=" << commands
                          << " timeouts=0 WKC_misses=" << wkc_misses
                          << "; sequential DM pairing, no sequence echo; no "
                             "hard-RT claim.\n";
                return 0;
            }
            new_command = false;
            if (active && !pending && Ns() >= next_send) {
                trace = {};
                trace.built = Ns();
                trace.exchange = exchange;
                auto* p = rx.data() + source;
                wire::Put32(p, ++batch);
                wire::Put32(p + 4, 20000);
                wire::Put16(p + 8, 1);
                p[10] = 3;
                p[11] = 0;
                const uint8_t disable[8] = {0xff, 0xff, 0xff, 0xff,
                                            0xff, 0xff, 0xff, 0xfd};
                const uint8_t zero[8] = {0, 0, 0, 0, 0, 0, 0, 0};
                const uint8_t bus[3] = {3, 3, 3};
                const uint32_t id[3] = {0x200, 0x000, 0x003};
                const uint8_t* data[3] = {zero, disable, disable};
                for (unsigned i = 0; i < 3; ++i) {
                    uint8_t* f = p + 16 + 16 * i;
                    f[0] = bus[i];
                    f[1] = 8;
                    f[2] = f[3] = 0;
                    wire::Put32(f + 4, id[i]);
                    std::memcpy(f + 8, data[i], 8);
                }
                trace.encoded = Ns();
                new_command = true;
                pending_wake = wake;
            }
            wire::Put32(rx.data() + 24, exchange);
            wire::Put32(rx.data() + 44, 1);
            for (const auto& e : output) {
                std::memcpy(data + e.domain, rx.data() + e.image, e.bytes);
            }
            if (new_command) {
                trace.output_copied = Ns();
            }
            ecrt_domain_queue(domain);
            if (new_command) {
                trace.queued = Ns();
            }
            last_pdo_send = Ns();
            if (new_command) {
                sent = last_pdo_send;
                next_send = sent + 10000000;
                pending = true;
                ++commands;
            }
            if (new_command) {
                trace.send_enter = Ns();
            }
            ecrt_master_send(master.ptr);
            if (new_command) {
                trace.send_return = Ns();
            }
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
            ReportUart(argv[2], uart_chunks);
            ReportCan(argv[2], can_history, can_history_count);
        } catch (const std::exception& report) {
            std::cerr << report.what() << std::endl;
        }
        return 1;
    }
}
