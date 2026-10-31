/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstdint>

#include "concurrency/published_snapshot.hpp"

namespace dust_control::devices {
struct OnboardImuSample {
    std::array<uint8_t, 25> raw{};
    float temperature_c = 0;
    uint64_t sample_us = 0;
    uint32_t sequence = 0;
    std::array<int16_t, 3> accel_raw{};
    std::array<int16_t, 3> gyro_raw{};
    std::array<float, 3> accel_mps2{};
    std::array<float, 3> gyro_rad_s{};
};

// HXY 传感器坐标系；未经实机验证，不替换 HI91 控制输入。
class OnboardImu {
   public:
    void Reset();
    void Consume(const std::array<uint8_t, 25>& raw, float temperature_c,
                 uint32_t sequence, uint64_t sample_us);
    OnboardImuSample Read() const;

   private:
    concurrency::PublishedSnapshot<OnboardImuSample> published_;
};
}  // namespace dust_control::devices
