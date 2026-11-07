/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <algorithm>
#include <array>
#include <cstdint>

#include "pdo_schema.hpp"

namespace dust_control::ethercat::device {
constexpr size_t kBytes = pdo::kRecordBytes;
constexpr size_t kPayloadBytes = pdo::kPayloadBytes;
constexpr size_t kMotorBatchBytes = pdo::kRxBytes;
constexpr auto kRoutes = pdo::kRoutes;
using Record = std::array<uint8_t, kBytes>;
using Payload = std::array<uint8_t, kPayloadBytes>;
enum class Direction { Rx, Tx };
enum class Device : uint8_t {
    RightWheel,
    RightJointB,
    RightJointD,
    LeftWheel,
    LeftJointB,
    LeftJointD
};
enum class Type : uint8_t {
    MotorFirst = 0x10,
    Discover = 0x30,
    Ready = 0x31,
    Arm = 0x32,
    Stop = 0x33,
    Status = 0x34
};
struct View {
    Type type;
    uint8_t sequence;
    Payload payload;
};
inline bool Decode(Direction direction, const uint8_t* data, size_t size,
                   View& view) {
    if (!data || size != kBytes) {
        return false;
    }
    const auto type = Type(data[0]);
    const auto motor = unsigned(data[0]) - unsigned(Type::MotorFirst);
    const bool control =
        direction == Direction::Rx
            ? type == Type::Discover || type == Type::Arm || type == Type::Stop
            : type == Type::Ready || type == Type::Status;
    if (motor >= kRoutes.size() && !control) {
        return false;
    }
    view = {type, data[1], {}};
    std::copy_n(data + 2, kPayloadBytes, view.payload.begin());
    return true;
}
inline Record Encode(Type type, uint8_t sequence, const Payload& payload) {
    Record record{uint8_t(type), sequence};
    std::copy(payload.begin(), payload.end(), record.begin() + 2);
    return record;
}
inline bool EncodeMotor(Device device, uint8_t sequence, const Payload& payload,
                        Record& record) {
    if (size_t(device) >= kRoutes.size()) {
        return false;
    }
    record = Encode(Type(unsigned(Type::MotorFirst) + unsigned(device)),
                    sequence, payload);
    return true;
}
inline bool DecodeMotorBatch(const uint8_t* data, size_t size,
                             uint8_t& sequence) {
    if (!data || size != kMotorBatchBytes) {
        return false;
    }
    for (unsigned i = 0; i < kRoutes.size(); ++i) {
        View view{};
        if (!Decode(Direction::Rx, data + i * kBytes, kBytes, view) ||
            unsigned(view.type) != unsigned(Type::MotorFirst) + i ||
            view.sequence != data[1]) {
            return false;
        }
    }
    sequence = data[1];
    return true;
}
}  // namespace dust_control::ethercat::device
