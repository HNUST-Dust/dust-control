/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstdint>

#include "concurrency/published_snapshot.hpp"

namespace dust::devices {
struct OnboardImuSample {
    std::array<uint8_t, 25> raw{};
    float temperature_c = 0;
    uint64_t sample_us = 0;
    uint32_t sequence = 0;
};

// 原始采样处理；安装坐标和量程未验证前不替换控制用 HI91。
class OnboardImu {
   public:
    void Reset() {
        published_.Publish({});
    }
    void Consume(const std::array<uint8_t, 25>& raw, float temperature_c,
                 uint32_t sequence, uint64_t sample_us) {
        published_.Publish({raw, temperature_c, sample_us, sequence});
    }
    OnboardImuSample Read() const {
        return published_.Read();
    }

   private:
    concurrency::PublishedSnapshot<OnboardImuSample> published_;
};
}  // namespace dust::devices
