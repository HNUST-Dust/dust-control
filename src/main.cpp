/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis_controller.h"
#include "runtime_config.hpp"
#include "motor_commands.hpp"
#include "usb_link.hpp"
#include "telemetry.hpp"
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <wbr/realtime_loop.hpp>
namespace {
volatile std::sig_atomic_t stopping = 0;
void Signal(int) { stopping = 1; }
} // namespace
int main(int argc, char **argv) {
  try {
    bool simulate = false, enable_output = false;
    double seconds = 0;
    std::string config_directory;
    int control_cpu = 3;
    int telemetry_port = 0, foxglove_port = 0;
    wbr::host::UsbOptions usb;
    for (int i = 1; i < argc; ++i) {
      const std::string option = argv[i];
      if (option == "--simulate") {
        simulate = true;
        continue;
      }
      if (option == "--foxglove") {
        foxglove_port = 8765;
        continue;
      }
      if (option == "--no-rt") {
        usb.realtime = false;
        continue;
      }
      if (option == "--enable-output") {
        enable_output = true;
        continue;
      }
      if (option == "--strict-hi91-crc") {
        usb.strict_hi91_crc = true;
        continue;
      }
      if (option == "--help") {
        std::cout
            << "chassis_controller [--cpu 3] [--usb-cpu 3] [--priority 80] "
               "[--config DIR] [--seconds N] [--foxglove] [--foxglove-port 8765]\n"
            << "  Default: disabled actuator policy; remote disabled pose must "
               "be observed\n"
            << "  after startup before --enable-output can honor the remote "
               "enable switch.\n"
            << "  --simulate --no-rt runs the real core without opening USB.\n";
        return 0;
      }
      if (++i == argc)
        throw std::runtime_error("Missing value for " + option);
      if (option == "--config")
        config_directory = argv[i];
      else if (option == "--cpu")
        control_cpu = std::stoi(argv[i]);
      else if (option == "--usb-cpu")
        usb.cpu = std::stoi(argv[i]);
      else if (option == "--priority")
        usb.priority = std::stoi(argv[i]);
      else if (option == "--foxglove-port")
        foxglove_port = std::stoi(argv[i]);
      else if (option == "--telemetry-port")
        telemetry_port = std::stoi(argv[i]);
      else if (option == "--seconds")
        seconds = std::stod(argv[i]);
      else
        throw std::runtime_error("Unknown option " + option);
    }
    if (control_cpu < 0 || control_cpu >= CPU_SETSIZE || usb.cpu < 0 ||
        usb.cpu >= CPU_SETSIZE || usb.priority < 1 || usb.priority > 99 ||
        seconds < 0 || !std::isfinite(seconds) ||
        telemetry_port < 0 || telemetry_port > 65535 ||
        foxglove_port < 0 || foxglove_port > 65535)
      throw std::runtime_error("Invalid runtime options");
    if (simulate && seconds == 0)
      seconds = 1;
    if (simulate && enable_output)
      throw std::runtime_error("Simulation cannot enable real outputs");
    if (!config_directory.empty()) {
      dust::config::Load(config_directory);
      std::cout << "Configuration loaded: " << config_directory << std::endl;
    }
    std::signal(SIGINT, Signal);
    std::signal(SIGTERM, Signal);
    wbr::host::SensorStore sensors;
    wbr::host::MotorCommands commands;
    modules::ChassisController controller(commands);
    std::unique_ptr<wbr::host::TelemetryPublisher> telemetry;
    if (telemetry_port || foxglove_port)
      telemetry = std::make_unique<wbr::host::TelemetryPublisher>(
          telemetry_port, control_cpu, usb.cpu, foxglove_port);
    std::unique_ptr<wbr::host::UsbLink> link;
    std::string error;
    if (usb.realtime && !wbr::realtime::ConfigureCurrentThread(
                            control_cpu, usb.priority, error))
      throw std::runtime_error(error);
    if (!simulate) {
      link = std::make_unique<wbr::host::UsbLink>(sensors, usb);
      if (!link->Open())
        throw std::runtime_error(link->error());
      std::cout << "Bridge capabilities=0x" << std::hex << link->capabilities()
                << std::dec << "; control IMU=HI91" << std::endl;
    }
    std::cout << "chassis_controller: 1 kHz, "
              << (simulate ? "offline" : "USB Bulk / CAN WBD")
              << ", output=" << (enable_output ? "remote-gated" : "disabled")
              << std::endl;
    const uint64_t start = wbr::realtime::MonotonicTimeNs();
    uint64_t release = start;
    const uint64_t finish =
        seconds ? start + uint64_t(seconds * 1e9) : UINT64_MAX;
    bool disabled_observed = false;
    unsigned late_cycles = 0;
    uint64_t next_telemetry = start, telemetry_sequence = 0, telemetry_drops = 0;
    wbr::host::TelemetrySample sample;
    while (!stopping && wbr::realtime::MonotonicTimeNs() < finish) {
      release += 1000000;
      if (!wbr::realtime::SleepUntilNs(release, error)) {
        if (stopping)
          break;
        if (link)
          link->Stop();
        throw std::runtime_error(error);
      }
      const uint64_t now = wbr::realtime::MonotonicTimeNs();
      // Never replay catch-up control ticks. A >=20 ms stall also exceeds
      // the MCU watchdog and requires a new session, not automatic rearm.
      if (now - release >= wbr::bridge::kCommandTimeoutUs * 1000ULL) {
        if (link)
          link->Stop();
        throw std::runtime_error("Control deadline exceeded bridge watchdog");
      }
      if (now - release >= 1000000) {
        const auto missed = (now - release) / 1000000;
        release += missed * 1000000;
        late_cycles += missed;
      }
      if (link && !link->healthy()) {
        link->Stop();
        throw std::runtime_error(link->error().empty()
                                     ? "Bridge sensor stream timeout"
                                     : link->error());
      }
      const auto snapshot = sensors.Read();
      disabled_observed |= snapshot.control.remote_disable_pose;
      commands.Begin();
      const auto tick_start = wbr::realtime::MonotonicTimeNs();
      const bool observe = telemetry && tick_start >= next_telemetry;
      controller.Tick(snapshot.control, tick_start / 1000,
                      enable_output && disabled_observed,
                      observe ? &sample.input : nullptr);
      if (link && !link->Send(commands.packet())) {
        link->Stop();
        throw std::runtime_error(link->error());
      }
      if (observe) {
        sample.monotonic_ns = tick_start;
        sample.control_duration_ns = wbr::realtime::MonotonicTimeNs() - tick_start;
        sample.sequence = ++telemetry_sequence;
        sample.missed_releases = late_cycles;
        sample.queue_drops = telemetry_drops;
        sample.state = controller.state();
        sample.simulated = simulate;
        if (!telemetry->Push(sample)) ++telemetry_drops;
        next_telemetry += ((tick_start - next_telemetry) / 10000000 + 1) *
                          10000000; // 100 Hz; skip late samples, never catch up.
      }
    }
    if (link) {
      link->Stop();
      link->Close();
    }
    std::cout << "Stopped; missed control releases=" << late_cycles
              << std::endl;
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
}
