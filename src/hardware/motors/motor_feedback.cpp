/* SPDX-License-Identifier: Apache-2.0 */
#include "motor_feedback.hpp"

#include "hardware/motors/dji_protocol.h"
#include "motor_map.h"

namespace dust::devices {
MotorFeedback DecodeMotorFeedback(unsigned index, const uint8_t* bytes,
                                  size_t size, uint64_t sample_us,
                                  const MotorConfig& config) {
    MotorFeedback sample{};
    sample.precise_timestamp_us = sample_us;
    if (index >= 6 || size != 8 || !bytes) {
        return sample;
    }
    if (index < 2) {
        protocols::DjiMotorFeedback decoded{};
        sample.valid = protocols::DecodeDjiFeedback(bytes, size, &decoded) == 0;
        sample.speed_rpm = static_cast<double>(decoded.omega);
        sample.velocity_rad_s = sample.speed_rpm * 0.10471975511965977;
    } else {
        protocols::DmMotorFeedbackNormal decoded{};
        sample.valid =
            protocols::DecodeDmFeedbackNormal(bytes, size, &decoded) == 0;
        sample.position_rad =
            protocols::DmFeedbackPosition(decoded, config.dm_range);
        sample.velocity_rad_s =
            protocols::DmFeedbackVelocity(decoded, config.dm_range);
        sample.torque_nm =
            protocols::DmFeedbackTorque(decoded, config.dm_range);
        sample.enabled = decoded.control_status == 1U;
    }
    return sample;
}
void MotorFeedbackDecoder::Consume(uint8_t bus, uint16_t id,
                                   std::span<const uint8_t> bytes,
                                   uint64_t sample_us, bool timestamp_valid) {
    for (unsigned motor = 0; motor < latest_.size(); ++motor) {
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
}  // namespace dust::devices
