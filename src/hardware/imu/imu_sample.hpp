/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <cstdint>

namespace dust_control::devices {
// DTG1 紧凑页解码后的设备物理量，不包含网关序号或伪造设备时间。
struct Hi91Measurement {
    float accel_g[3] = {};
    float gyro_dps[3] = {};
    float roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
};
// HI91 发布的控制用物理量样本，时间戳使用主机单调时钟。
struct ImuSample {
    uint32_t sequence = 0U;
    uint64_t precise_timestamp_us = 0U;
    bool valid = false;
    float pitch_deg = 0.0F;
    float roll_deg = 0.0F;
    float yaw_deg = 0.0F;
    float pitch_rate_rad_s = 0.0F;
    float roll_rate_rad_s = 0.0F;
    float yaw_rate_rad_s = 0.0F;
    float accel_g[3] = {};
    float vertical_accel_g = 0.0F;
};

}  // namespace dust_control::devices
