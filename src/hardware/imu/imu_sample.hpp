/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <cstdint>

namespace dust::devices {
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

}  // namespace dust::devices
