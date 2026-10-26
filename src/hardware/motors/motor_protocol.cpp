/* SPDX-License-Identifier: Apache-2.0 */
#include "motor_protocol.hpp"

#include "dm_protocol.h"
#include "hardware/motors/dji_protocol.h"

namespace dust_control::devices {
MotorFeedback DecodeMotorFeedback(unsigned index, const uint8_t* bytes,
                                  size_t size, uint64_t sample_us,
                                  const MotorConfig& config) {
    MotorFeedback sample{};
    sample.precise_timestamp_us = sample_us;
    if (index >= 6 || !config.present[index] || size != 8 || !bytes) {
        return sample;
    }
    return config.protocols[index] == MotorProtocol::kDji
               ? dust_control::protocols::DecodeDjiMotorState(bytes, size,
                                                              sample_us)
               : dust_control::protocols::DecodeDmMotorState(bytes, size,
                                                             sample_us);
}

void MotorFeedbackDecoder::Consume(uint8_t bus, uint16_t id,
                                   std::span<const uint8_t> bytes,
                                   uint64_t sample_us, bool timestamp_valid) {
    for (unsigned motor = 0; motor < latest_.size(); ++motor) {
        if (!config_.present[motor]) {
            continue;
        }
        const auto& address = config_.addresses[motor];
        if (bus != address.bus || id != address.feedback_id) {
            continue;
        }
        auto& sample = latest_[motor];
        sample.valid = timestamp_valid && bytes.size() == 8;
        sample.precise_timestamp_us = sample.valid ? sample_us : 0;
        if (sample.valid) {
            sample = DecodeMotorFeedback(motor, bytes.data(), bytes.size(),
                                         sample_us, config_);
        }
    }
    published_.Publish(latest_);
}
std::optional<CanTransmitBatch> MotorEncoder::Encode(
    const MotorTargets& targets) const {
    CanTransmitBatch batch;
    // 与现有主机发送顺序一致：左 W/B/D，然后右 W/B/D；固件按 FIFO 转发。
    for (const unsigned index : {0U, 2U, 3U, 1U, 4U, 5U}) {
        if (!config_.present[index]) {
            continue;
        }
        const auto& target = targets[index];
        if (target.mode == MotorMode::kNone) {
            continue;
        }
        auto& frame = batch.frames[batch.count];
        const auto& address = config_.addresses[index];
        frame.bus = address.bus;
        frame.id = address.command_id;
        const bool encoded =
            config_.protocols[index] == MotorProtocol::kDji
                ? dust_control::protocols::EncodeDjiMotorTarget(
                      target, address.command_slot,
                      config_.wheel_command_sign[index], frame.bytes.data())
                : dust_control::protocols::EncodeDmMotorTarget(
                      target, frame.bytes.data());
        if (!encoded) {
            return std::nullopt;
        }
        ++batch.count;
    }
    return batch;
}
}  // namespace dust_control::devices
