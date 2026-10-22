/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <dust_adapter/protocol.hpp>
namespace wbr::bridge {
using namespace dust::wire;
using Commands = dust::wire::Request;
constexpr uint32_t kCommandTimeoutUs = 20000; // application-selected MCU lease
inline bool HasHeader(const uint8_t *p, size_t length, Kind kind) {
  return dust::wire::HasHeader(p, length) && p[9] == uint8_t(kind);
}
inline bool ValidCommands(const Commands &packet, uint32_t session) {
  return HasHeader(packet.data(), packet.size(), Kind::Transmit) &&
         U32(packet.data() + 12) == session && ValidRecords(packet);
}
// Uplink fixture/helper: 16-byte raw record followed by RX timestamp and per-bus sequence.
inline void CanRecord(uint8_t *p, uint8_t bus, uint32_t id, const uint8_t *bytes,
                      uint8_t length, uint32_t timestamp = 0, uint32_t sequence = 0) {
  dust::wire::Record record{};
  record.channel = bus; record.id = id; record.length = length;
  std::memcpy(record.data, bytes, length);
  Encode(p, record); Put32(p + 16, timestamp); Put32(p + 20, sequence);
}
} // namespace wbr::bridge
