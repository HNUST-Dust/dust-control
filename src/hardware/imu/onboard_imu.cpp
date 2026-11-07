/* SPDX-License-Identifier: Apache-2.0 */
#include "onboard_imu.hpp"

namespace dust_control::devices {
void OnboardImu::Reset() {
    published_.Publish({});
}
void OnboardImu::Consume(const std::array<int16_t, 3>& accel_raw,
                         const std::array<int16_t, 3>& gyro_raw,
                         float temperature_c, uint32_t sequence,
                         uint64_t sample_us) {
    OnboardImuSample sample{};
    sample.accel_raw = accel_raw;
    sample.gyro_raw = gyro_raw;
    sample.temperature_c = temperature_c;
    sample.sample_us = sample_us;
    sample.sequence = sequence;
    constexpr float accel_scale = 9.80665F / 4096.0F;
    constexpr float gyro_scale = 0.017453292519943295F / 16.384F;
    for (size_t axis = 0; axis < 3; ++axis) {
        sample.accel_mps2[axis] = sample.accel_raw[axis] * accel_scale;
        sample.gyro_rad_s[axis] = sample.gyro_raw[axis] * gyro_scale;
    }
    published_.Publish(sample);
}
OnboardImuSample OnboardImu::Read() const {
    return published_.Read();
}
}  // namespace dust_control::devices
