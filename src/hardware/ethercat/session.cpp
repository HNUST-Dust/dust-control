/* SPDX-License-Identifier: Apache-2.0 */
#include "session.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <stdexcept>

#include "device_pdo.hpp"
#include "wire.hpp"

namespace dust_control::host {
namespace dw = ethercat::device;
namespace ds = ethercat::sensor;
namespace pdo = ethercat::pdo;
namespace w = ethercat::wire;
EthercatSession::EthercatSession(const ethercat::AdapterPdoLayout& layout,
                                 const devices::MotorConfig& motors)
    : motors_(motors),
      motor_encoder_(motors),
      decode_motors_(std::any_of(motors.present.begin(), motors.present.end(),
                                 [](bool present) { return present; })) {
    if (layout.rx.bytes != pdo::kRxBytes || layout.tx.bytes != pdo::kTxBytes ||
        layout.layout_signature != pdo::kSignature ||
        layout.cycle_time_us != pdo::kCycleTimeUs) {
        throw std::invalid_argument("DTG1 PDO layout mismatch");
    }
    if (decode_motors_) {
        for (const auto& route : pdo::kRoutes) {
            bool found = false;
            for (size_t i = 0; i < motors.present.size(); ++i) {
                found |= motors.present[i] &&
                         motors.addresses[i].bus == route.bus &&
                         motors.addresses[i].command_id == route.command &&
                         motors.addresses[i].feedback_id == route.feedback;
            }
            if (!found) {
                throw std::invalid_argument("DTG1 motor route mismatch");
            }
        }
    }
}
bool EthercatSession::FailInput(const char* reason) {
    transmitter_.Fail(reason);
    return false;
}
bool EthercatSession::Receive(std::span<const uint8_t> image, uint64_t now,
                              uint64_t session) {
    if (image.size() != pdo::kTxBytes) {
        return FailInput("DTG1 TxPDO length mismatch");
    }
    const auto* p = image.data();
    remote_.PollTimeout(now / 1000);
    if (p[0] == uint8_t(dw::Type::Ready) || p[0] == uint8_t(dw::Type::Status)) {
        dw::View view{};
        if (!dw::Decode(dw::Direction::Tx, p, dw::kBytes, view)) {
            return FailInput("DTG1 invalid status");
        }
        if (ds::Get32(view.payload.data()) != uint32_t(session)) {
            return true;
        }
        if (view.type == dw::Type::Ready) {
            if (active_) {
                return FailInput("DTG1 session regressed; restart required");
            }
            discovered_ = true;
        } else {
            if (view.payload[4] == 3) {
                return FailInput("DTG1 slave protected");
            }
            if (view.payload[4] == 2) {
                discovered_ = active_ = true;
            } else if (active_) {
                return FailInput("DTG1 session lost");
            }
        }
        return true;
    }
    if (!active_) {
        return true;
    }
    for (size_t i = 0; i < pdo::kRoutes.size(); ++i) {
        const auto* record = p + pdo::kMotorsOffset + i * dw::kBytes;
        if (std::all_of(record, record + dw::kBytes,
                        [](uint8_t value) { return value == 0; })) {
            continue;
        }
        dw::View view{};
        if (!dw::Decode(dw::Direction::Tx, record, dw::kBytes, view) ||
            unsigned(view.type) != unsigned(dw::Type::MotorFirst) + i) {
            return FailInput("DTG1 invalid motor page");
        }
        if (state_.have_motor[i] && state_.motor_sequence[i] == view.sequence) {
            continue;
        }
        state_.have_motor[i] = true;
        state_.motor_sequence[i] = view.sequence;
        const auto& route = pdo::kRoutes[i];
        if (decode_motors_) {
            motors_.Consume(route.bus, route.feedback, view.payload, now / 1000,
                            true);
        }
    }
    // 板载页只有 28 字节；其余传输区不属于该页，解码前在本地补零。
    ds::Page board_page{};
    std::copy_n(p + pdo::kOnboardImuOffset, pdo::kOnboardImuBytes,
                board_page.begin());
    if (board_page[0]) {
        ds::Fragment view{};
        if (!ds::Decode(board_page.data(), board_page.size(), view) ||
            view.kind != ds::Kind::ImuCompact) {
            return FailInput("DTG1 invalid onboard IMU page");
        }
        if (!state_.have_imu || w::Newer(view.sequence, state_.imu_sequence)) {
            const float temperature =
                std::bit_cast<float>(w::U32(view.data.data() + 12));
            if (!std::isfinite(temperature)) {
                return FailInput("DTG1 invalid IMU temperature");
            }
            std::array<int16_t, 3> accel{}, gyro{};
            for (size_t axis = 0; axis < 3; ++axis) {
                accel[axis] = w::I16Be(view.data.data() + axis * 2);
                gyro[axis] = w::I16Be(view.data.data() + 6 + axis * 2);
            }
            onboard_imu_.Consume(accel, gyro, temperature, view.sequence,
                                 now / 1000);
            state_.imu_sequence = view.sequence;
            state_.have_imu = true;
        }
    }
    const auto* sensor = p + pdo::kHi91Offset;
    if (sensor[0] == pdo::kHi91PageType) {
        const auto sequence = w::U32(sensor + 1);
        if (!state_.have_hi91 || sequence != state_.hi91_sequence) {
            if (state_.have_hi91 && !w::Newer(sequence, state_.hi91_sequence)) {
                return FailInput("DTG1 HI91 sequence regressed");
            }
            const auto* fields = sensor + pdo::kHi91FieldsOffset;
            auto read_float = [fields](size_t offset) {
                return std::bit_cast<float>(w::U32(fields + offset));
            };
            devices::Hi91Measurement sample{};
            for (size_t axis = 0; axis < 3; ++axis) {
                sample.accel_g[axis] = read_float(axis * 4);
                sample.gyro_dps[axis] = read_float(12 + axis * 4);
            }
            sample.roll_deg = read_float(24);
            sample.pitch_deg = read_float(28);
            sample.yaw_deg = read_float(32);
            imu_.Consume(sample, now / 1000);
            state_.hi91_sequence = sequence;
            state_.have_hi91 = true;
        }
    } else if (sensor[0]) {
        ds::Fragment view{};
        if (!ds::Decode(sensor, pdo::kHi91Bytes, view) ||
            view.kind != ds::Kind::Sbus) {
            return FailInput("DTG1 invalid UART page");
        }
        if (!state_.have_sbus || w::Newer(view.sequence, state_.sbus_ack)) {
            if (view.discontinuity) {
                remote_.Discontinuity();
            }
            remote_.Consume({view.data.data(), view.length}, now / 1000);
            state_.sbus_ack = view.sequence;
            state_.have_sbus = true;
            state_.ack_repeats = 2;
        } else if (view.sequence == state_.sbus_ack) {
            state_.ack_repeats = 2;
        }
    }
    return true;
}
bool EthercatSession::Encode(std::span<uint8_t> image, uint64_t now,
                             uint64_t session, uint32_t flags) {
    if (image.size() != pdo::kRxBytes || !uint32_t(session)) {
        return FailInput("DTG1 invalid output/session");
    }
    std::fill(image.begin(), image.end(), 0);
    if (flags != w::kStop && flags != w::kDiscover && flags != w::kArm) {
        return FailInput("DTG1 invalid control flags");
    }
    if (flags == w::kArm && active_) {
        EthercatTransmitter::Batch batch;
        if (!transmitter_.ReadBatch(now, batch)) {
            return FailInput("DTG1 motor command expired");
        }
        if (batch.valid && (!state_.have_command ||
                            batch.sequence != state_.command_source_sequence)) {
            if (batch.count != pdo::kRoutes.size()) {
                return FailInput("DTG1 requires six motor commands");
            }
            std::array<bool, 6> seen{};
            ++state_.sequence;
            for (size_t i = 0; i < batch.count; ++i) {
                const auto& frame = batch.frames[i];
                const auto route =
                    std::find_if(pdo::kRoutes.begin(), pdo::kRoutes.end(),
                                 [&](const auto& value) {
                                     return value.bus == frame.bus &&
                                            value.command == frame.id;
                                 });
                if (route == pdo::kRoutes.end()) {
                    return FailInput("DTG1 unknown CAN route");
                }
                const auto index = size_t(route - pdo::kRoutes.begin());
                if (seen[index]) {
                    return FailInput("DTG1 duplicate motor command");
                }
                seen[index] = true;
                dw::Record record{};
                dw::EncodeMotor(dw::Device(index), state_.sequence, frame.bytes,
                                record);
                std::copy(record.begin(), record.end(),
                          state_.held.begin() + index * dw::kBytes);
            }
            state_.have_command = true;
            state_.command_source_sequence = batch.sequence;
            std::copy(state_.held.begin(), state_.held.end(), image.begin());
            return true;
        }
        if (state_.have_command && !state_.ack_repeats) {
            std::copy(state_.held.begin(), state_.held.end(), image.begin());
            return true;
        }
    }
    dw::Payload payload{};
    w::Put32(payload.data(), uint32_t(session));
    payload[5] = state_.ack_repeats ? 2 : 0;
    payload[6] = 8 | 16 | 32 | 64;
    payload[7] = 8 | 16 | (2 << 5);
    const auto type = flags == w::kStop       ? dw::Type::Stop
                      : flags == w::kDiscover ? dw::Type::Discover
                                              : dw::Type::Arm;
    const auto record = dw::Encode(type, ++state_.sequence, payload);
    std::copy(record.begin(), record.end(), image.begin());
    w::Put32(image.data() + 14, state_.sbus_ack);
    if (state_.ack_repeats) {
        --state_.ack_repeats;
    }
    return true;
}
}  // namespace dust_control::host
