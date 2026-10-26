/* SPDX-License-Identifier: Apache-2.0 */
#include "onboard_imu.hpp"

namespace dust_control::devices {
void OnboardImu::Reset() {
    published_.Publish({});
}
void OnboardImu::Consume(const std::array<uint8_t, 25>& raw,
                         float temperature_c, uint32_t sequence,
                         uint64_t sample_us) {
    published_.Publish({raw, temperature_c, sample_us, sequence});
}
OnboardImuSample OnboardImu::Read() const {
    return published_.Read();
}
}  // namespace dust_control::devices
