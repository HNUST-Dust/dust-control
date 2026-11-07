/* SPDX-License-Identifier: Apache-2.0 */
#include "hi91_imu.hpp"

#include <algorithm>
#include <cmath>

namespace dust_control::devices {
void Hi91Imu::Reset() {
    sequence_ = 0;
    published_.Publish({});
}
bool Hi91Imu::Consume(const Hi91Measurement& measurement, uint64_t sample_us) {
    constexpr double kMaximumValidPitchDeg = 100.0;
    if (!std::isfinite(measurement.pitch_deg) ||
        std::abs(measurement.pitch_deg) > kMaximumValidPitchDeg ||
        !std::isfinite(measurement.roll_deg) ||
        !std::isfinite(measurement.yaw_deg)) {
        return false;
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (!std::isfinite(measurement.gyro_dps[i]) ||
            !std::isfinite(measurement.accel_g[i])) {
            return false;
        }
    }
    ImuSample sample{};
    sample.sequence = ++sequence_;
    sample.precise_timestamp_us = sample_us;
    sample.valid = true;
    sample.pitch_deg = measurement.pitch_deg;
    sample.roll_deg = measurement.roll_deg;
    sample.yaw_deg = measurement.yaw_deg;
    constexpr float radians = 0.01745329251994329577F;
    // 保留现有安装映射：陀螺仪 X 对应 pitch，Y 对应 roll。
    sample.pitch_rate_rad_s = measurement.gyro_dps[0] * radians;
    sample.roll_rate_rad_s = measurement.gyro_dps[1] * radians;
    sample.yaw_rate_rad_s = measurement.gyro_dps[2] * radians;
    std::copy_n(measurement.accel_g, 3, sample.accel_g);
    sample.vertical_accel_g = measurement.accel_g[2];
    published_.Publish(sample);
    return true;
}
}  // namespace dust_control::devices
