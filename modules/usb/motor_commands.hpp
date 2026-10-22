/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "frame_sink.h"
#include <cerrno>
#include <chassis_controller/motor_map.h>
#include <wbr_bridge/bridge_protocol.hpp>
namespace wbr::host {
class MotorCommands final : public modules::FrameSink {
public:
  void Begin() { present_ = {}; records_ = {}; }
  int Submit(platform::CanTxSlot slot, uint8_t bus, uint16_t id,
             const uint8_t *data, size_t length) override {
    const auto index = static_cast<unsigned>(slot);
    if (index >= 6 || length != 8 || !data) return -EINVAL;
    const auto &motor = platform::motor_map::kMotors[index];
    if (motor.bus != bus || motor.command_id != id) return -EINVAL;
    auto &record = records_[index];
    record.channel = bus; record.id = id; record.length = length;
    std::memcpy(record.data, data, length);
    present_[index] = true;
    return 0;
  }
  bridge::Commands packet() const {
    bridge::Commands packet{};
    // Device-specific scheduling stays on the host. Firmware forwards FIFO order.
    for (unsigned slot : {0U, 2U, 3U, 1U, 4U, 5U}) {
      if (!present_[slot]) continue;
      bridge::Encode(packet.data() + 32 + 16 * packet[10]++, records_[slot]);
    }
    return packet;
  }
private:
  std::array<bridge::Record, 6> records_{};
  std::array<bool, 6> present_{};
};
} // namespace wbr::host
