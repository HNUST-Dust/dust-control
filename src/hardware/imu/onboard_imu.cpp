/* SPDX-License-Identifier: Apache-2.0 */
#include "onboard_imu.hpp"

namespace dust_control::devices {
void OnboardImu::Reset() {
    published_.Publish({});
}
void OnboardImu::Consume(const std::array<uint8_t, 25>& raw,
                         float temperature_c, uint32_t sequence,
                         uint64_t sample_us) {
    OnboardImuSample sample{};
    sample.raw = raw;
    sample.temperature_c = temperature_c;
    sample.sample_us = sample_us;
    sample.sequence = sequence;
    constexpr float accel_scale = 9.80665F / 4096.0F;
    constexpr float gyro_scale = 0.017453292519943295F / 16.384F;
    auto read_signed = [&raw](size_t i) -> int16_t {
        const uint16_t value = (uint16_t(raw[i]) << 8U) | raw[i + 1U];
        return value < 0x8000U ? int16_t(value) : int16_t(int32_t(value) - 65536);
    };
    for (size_t axis = 0; axis < 3; ++axis) {
        sample.accel_raw[axis] = read_signed(1U + axis * 2U);
        sample.gyro_raw[axis] = read_signed(7U + axis * 2U);
        sample.accel_mps2[axis] = sample.accel_raw[axis] * accel_scale;
        sample.gyro_rad_s[axis] = sample.gyro_raw[axis] * gyro_scale;
    }
    published_.Publish(sample);
}
OnboardImuSample OnboardImu::Read() const {
    return published_.Read();
}
}  // namespace dust_control::devices
