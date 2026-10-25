/* SPDX-License-Identifier: Apache-2.0 */
#include "imu_config.hpp"

#include "yaml_validation.hpp"

namespace dust::devices {
ImuConfig LoadImuConfig(const std::string& file) {
    using namespace dust::yaml_validation;
    const auto root = LoadSensorFile(file);
    ValidateKeys(root["hi91"], {"freshness_deadline_us"});
    return {ParseFreshnessDeadline(root["hi91"]["freshness_deadline_us"],
                                   UINT32_MAX - 1ULL)};
}
}  // namespace dust::devices
