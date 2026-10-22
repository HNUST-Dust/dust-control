/* SPDX-License-Identifier: Apache-2.0 */
#include "telemetry.hpp"
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <locale>
#include <future>
#include <sstream>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <iostream>
#include <memory>
#include <wbr/realtime_loop.hpp>
#ifdef WBR_ENABLE_FOXGLOVE
#include <foxglove/channel.hpp>
#include <foxglove/websocket.hpp>
#include <foxglove/error.hpp>
#include "telemetry_schema.hpp"
#endif

namespace wbr::host {
namespace {
std::string Encode(const TelemetrySample &sample, bool connected = true) {
  const auto &in = sample.input;
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out.precision(12);
  out << "{\"version\":1,\"sequence\":" << sample.sequence
      << ",\"monotonic_ns\":" << sample.monotonic_ns
      << ",\"simulated\":" << (sample.simulated ? "true" : "false")
      << ",\"imu_fresh\":" << (in.imu_fresh ? "true" : "false")
      << ",\"remote_fresh\":" << (in.remote_fresh ? "true" : "false")
      << ",\"feedback_valid\":" << (in.feedback_valid ? "true" : "false")
      << ",\"motor_stale_mask\":" << unsigned(in.motor_stale_mask)
      << ",\"state\":" << unsigned(sample.state)
      << ",\"missed_releases\":" << sample.missed_releases
      << ",\"queue_drops\":" << sample.queue_drops
      << ",\"source_connected\":" << (connected ? "true" : "false")
      << ",\"transport_age_ms\":"
      << (wbr::realtime::MonotonicTimeNs() - sample.monotonic_ns) * 1e-6
      << ",\"control_duration_us\":" << sample.control_duration_ns * 1e-3;
  const auto number = [&](const char *name, double value, bool valid) {
    out << ",\"" << name << "\":";
    if (valid && std::isfinite(value)) out << value;
    else out << "null";
  };
  number("imu_age_us", double(in.imu_age_us), in.imu_age_us != UINT64_MAX);
  number("pitch_deg", in.imu.pitch_deg, in.imu_fresh);
  number("roll_deg", in.imu.roll_deg, in.imu_fresh);
  number("yaw_deg", in.imu.yaw_deg, in.imu_fresh);
  const auto leg = [&](const char *side, const modules::LegKinematics &value,
                       unsigned motor_mask, unsigned invalid_bit) {
    const bool fresh = !(in.motor_stale_mask & motor_mask) &&
                       !(in.feedback_invalid_mask & invalid_bit);
    out << ",\"" << side << "_leg_fresh\":" << (fresh ? "true" : "false");
    number((std::string(side) + "_length_m").c_str(), value.length, fresh);
    // Native kinematics has +z upwards; foot below hip has negative hz.
    number((std::string(side) + "_height_m").c_str(), -value.hz, fresh);
    number((std::string(side) + "_angle_rad").c_str(), value.angle, fresh);
    number((std::string(side) + "_length_rate_m_s").c_str(), value.length_rate, fresh);
  };
  leg("left", in.leg.left, 0x0c, 4);
  leg("right", in.leg.right, 0x30, 8);
  out << '}';
  return out.str();
}
#ifdef WBR_ENABLE_FOXGLOVE
// Created on the non-RT worker AFTER its CPU/scheduler configuration. SDK-owned
// network threads therefore inherit SCHED_OTHER and the non-control CPU set.
class NativeFoxglove {
public:
  explicit NativeFoxglove(unsigned port) {
    const auto mono = wbr::realtime::MonotonicTimeNs();
    const auto epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    epoch_offset_ = epoch - static_cast<int64_t>(mono);
    foxglove::Schema schema;
    schema.name = "wbr.ChassisTelemetry";
    schema.encoding = "jsonschema";
    schema.data = reinterpret_cast<const std::byte *>(kTelemetrySchema);
    schema.data_len = sizeof(kTelemetrySchema) - 1;
    auto channel = foxglove::RawChannel::create("/wbr/chassis", "json", schema);
    if (!channel.has_value())
      throw std::runtime_error(foxglove::strerror(channel.error()));
    channel_ = std::make_unique<foxglove::RawChannel>(std::move(channel.value()));
    foxglove::WebSocketServerOptions options;
    options.name = "WBR chassis (native, read only)";
    options.host = "127.0.0.1";
    options.port = static_cast<uint16_t>(port);
    options.message_backlog_size = 128;
    auto server = foxglove::WebSocketServer::create(std::move(options));
    if (!server.has_value())
      throw std::runtime_error("Foxglove port " + std::to_string(port) + ": " +
                               foxglove::strerror(server.error()));
    server_ = std::make_unique<foxglove::WebSocketServer>(std::move(server.value()));
    std::cout << "Foxglove native: ws://127.0.0.1:" << port << std::endl;
  }
  ~NativeFoxglove() { if (server_) server_->stop(); }
  void Log(const std::string &json, uint64_t monotonic_ns) {
    channel_->log(reinterpret_cast<const std::byte *>(json.data()), json.size(),
                  static_cast<uint64_t>(static_cast<int64_t>(monotonic_ns) + epoch_offset_));
  }
private:
  int64_t epoch_offset_ = 0;
  std::unique_ptr<foxglove::RawChannel> channel_;
  std::unique_ptr<foxglove::WebSocketServer> server_;
};
#endif
} // namespace
TelemetryPublisher::TelemetryPublisher(unsigned port, int control_cpu, int usb_cpu,
                                         unsigned foxglove_port) {
  const int fd = port ? socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0) : -1;
  if (port && fd < 0) throw std::runtime_error("Cannot create telemetry UDP socket");
  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(static_cast<uint16_t>(port));
  destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  cpu_set_t cpus{};
  if (pthread_getaffinity_np(pthread_self(), sizeof(cpus), &cpus) != 0) {
    if (fd >= 0) close(fd);
    throw std::runtime_error("Cannot read telemetry CPU affinity");
  }
  CPU_CLR(control_cpu, &cpus);
  CPU_CLR(usb_cpu, &cpus);
  if (CPU_COUNT(&cpus) == 0) {
    if (fd >= 0) close(fd);
    throw std::runtime_error("Telemetry needs a CPU outside the control/USB CPUs");
  }
  std::promise<std::string> ready;
  auto started = ready.get_future();
  try {
    worker_ = std::thread([this, fd, destination, cpus, foxglove_port, &ready] {
      bool initialized = false;
      try {
        sched_param normal{};
        if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &normal) != 0 ||
            pthread_setaffinity_np(pthread_self(), sizeof(cpus), &cpus) != 0)
          throw std::runtime_error("Cannot configure non-RT telemetry thread");
#ifdef WBR_ENABLE_FOXGLOVE
        std::unique_ptr<NativeFoxglove> native;
        if (foxglove_port) native = std::make_unique<NativeFoxglove>(foxglove_port);
#else
        if (foxglove_port)
          throw std::runtime_error("Rebuild with WBR_ENABLE_FOXGLOVE=ON");
#endif
        ready.set_value("");
        initialized = true;
        TelemetrySample sample, latest;
        uint64_t last_publish = 0;
        bool have_sample = false;
        while (!stop_.load(std::memory_order_relaxed)) {
          bool found = false;
          while (queue_.Pop(sample)) { latest = sample; found = have_sample = true; }
          const auto now = wbr::realtime::MonotonicTimeNs();
          const bool connected = have_sample && now - latest.monotonic_ns <= 200000000;
          const bool stale_update = have_sample && !connected && now - last_publish >= 100000000;
          if (found || stale_update) {
            auto observed = latest;
            if (!connected) {
              observed.input.imu_fresh = observed.input.remote_fresh = false;
              observed.input.feedback_valid = false;
              observed.input.motor_stale_mask = 0x3f;
              observed.input.feedback_invalid_mask = 0xff;
              observed.input.imu_age_us = UINT64_MAX;
            }
            const auto json = Encode(observed, connected);
            if (fd >= 0)
              (void)sendto(fd, json.data(), json.size(), MSG_DONTWAIT,
                           reinterpret_cast<const sockaddr *>(&destination), sizeof(destination));
#ifdef WBR_ENABLE_FOXGLOVE
            if (native) native->Log(json, connected ? latest.monotonic_ns : now);
#endif
            last_publish = now;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
      } catch (const std::exception &error) {
        if (!initialized) ready.set_value(error.what());
        else std::cerr << "Telemetry stopped: " << error.what() << std::endl;
      }
      if (fd >= 0) close(fd);
    });
  } catch (...) { if (fd >= 0) close(fd); throw; }
  const auto error = started.get();
  if (!error.empty()) {
    worker_.join();
    throw std::runtime_error(error);
  }
}
TelemetryPublisher::~TelemetryPublisher() {
  stop_.store(true, std::memory_order_relaxed);
  if (worker_.joinable()) worker_.join();
}
} // namespace wbr::host
