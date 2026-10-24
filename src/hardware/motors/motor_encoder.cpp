/* SPDX-License-Identifier: Apache-2.0 */
#include "motor_encoder.hpp"

#include <algorithm>
#include <cmath>

#include "hardware/motors/dji_protocol.h"
#include "motor_map.h"

namespace dust::devices {
namespace {

bool EncodeWheel(unsigned index,
                 const platform::motor_map::MotorAddress& address,
                 const MotorConfig& config, double torque_nm, uint8_t* bytes) {
    if (address.command_slot >= 4) {
        return false;
    }
    const double sign = config.wheel_command_sign[index];
    const double requested = sign * torque_nm;
    const double current =
        std::isfinite(requested)
            ? std::clamp(
                  requested * config.dji_current_per_nm,
                  -static_cast<double>(config.dji_protocol_current_limit),
                  static_cast<double>(config.dji_protocol_current_limit))
            : 0.0;
    int16_t currents[4]{};
    currents[address.command_slot] = static_cast<int16_t>(current);
    return protocols::EncodeDjiCurrentFrame0x200(currents, bytes) == 0;
}

bool EncodeJoint(const MotorTarget& target, const MotorConfig& config,
                 uint8_t* bytes) {
    if (target.mode == MotorMode::kTorque) {
        protocols::DmMitCommand command{};
        command.torque = static_cast<float>(target.torque_nm);
        return protocols::PackDmMitCommand(&command, &config.dm_range, bytes) ==
               0;
    }
    protocols::DmControlCommand command;
    switch (target.mode) {
        case MotorMode::kDisable:
            command = protocols::DmControlCommand::kExit;
            break;
        case MotorMode::kClearError:
            command = protocols::DmControlCommand::kClearError;
            break;
        case MotorMode::kEnable:
            command = protocols::DmControlCommand::kEnter;
            break;
        default:
            return false;
    }
    return protocols::GetDmControlCommandFrame(command, bytes) == 0;
}
}  // namespace

std::optional<CanTransmitBatch> MotorEncoder::Encode(
    const MotorTargets& targets) const {
    CanTransmitBatch batch;
    // 与现有主机发送顺序一致：左 W/B/D，然后右 W/B/D；固件按 FIFO 转发。
    for (const unsigned index : {0U, 2U, 3U, 1U, 4U, 5U}) {
        const auto& target = targets[index];
        if (target.mode == MotorMode::kNone) {
            continue;
        }
        auto& frame = batch.frames[batch.count];
        const auto& address = config_.addresses[index];
        frame.bus = address.bus;
        frame.id = address.command_id;
        const bool encoded =
            index < 2 ? target.mode == MotorMode::kTorque &&
                            EncodeWheel(index, address, config_,
                                        target.torque_nm, frame.bytes.data())
                      : EncodeJoint(target, config_, frame.bytes.data());
        if (!encoded) {
            return std::nullopt;
        }
        ++batch.count;
    }
    return batch;
}
}  // namespace dust::devices
