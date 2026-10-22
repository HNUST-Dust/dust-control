/* SPDX-License-Identifier: Apache-2.0 */
#include "telemetry.hpp"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <wbr/realtime_loop.hpp>
using namespace wbr::host;
void Require(bool ok) { if (!ok) throw std::runtime_error("Telemetry contract failed"); }
int main(int argc, char **argv) {
  // Full queue drops without overwriting unread samples; wrap preserves order.
  TelemetryQueue queue;
  TelemetrySample sample;
  for (unsigned i = 0; i < 63; ++i) { sample.sequence = i; Require(queue.Push(sample)); }
  Require(!queue.Push(sample));
  for (unsigned i = 0; i < 63; ++i) { Require(queue.Pop(sample)); Require(sample.sequence == i); }
  Require(!queue.Pop(sample));
  std::thread producer([&] {
    for (unsigned i = 0; i < 20000; ++i) {
      TelemetrySample value; value.sequence = i;
      while (!queue.Push(value)) std::this_thread::yield();
    }
  });
  for (unsigned i = 0; i < 20000; ++i) {
    while (!queue.Pop(sample)) std::this_thread::yield();
    Require(sample.sequence == i);
  }
  producer.join();
  if (argc >= 2) {
    const bool native = argc == 3;
    TelemetryPublisher publisher(native ? 0 : std::stoul(argv[1]), 3, 3,
                                 native ? std::stoul(argv[1]) : 0);
    sample = {};
    sample.input.imu_fresh = true;
    sample.input.feedback_valid = true;
    sample.input.imu.pitch_deg = 12.5;
    sample.input.imu.roll_deg = -3.25;
    sample.input.imu.yaw_deg = 70;
    sample.input.leg.left.length = .25;
    sample.input.leg.left.hz = -.23;
    sample.input.leg.right.length = .26;
    sample.input.leg.right.hz = -.24;
    for (unsigned i = 0; i < 30; ++i) {
      sample.sequence = i + 1;
      sample.monotonic_ns = wbr::realtime::MonotonicTimeNs();
      // Second half: stale left joints, fresh right, invalid IMU.
      if (i >= 15) {
        sample.input.imu_fresh = false;
        sample.input.motor_stale_mask = 0x0c;
        sample.input.feedback_valid = false;
        sample.input.imu.yaw_deg = std::numeric_limits<float>::quiet_NaN();
      }
      Require(publisher.Push(sample));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (native) std::this_thread::sleep_for(std::chrono::milliseconds(400));
  }
  std::cout << "Telemetry queue saturation, wrap and concurrent ordering passed\n";
}
