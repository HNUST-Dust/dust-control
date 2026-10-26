/* SPDX-License-Identifier: Apache-2.0 */
#include "remote_config.hpp"

#include "yaml_validation.hpp"

namespace dust_control::devices {
RemoteConfig LoadRemoteConfig(const std::string& file) {
    using namespace dust_control::yaml_validation;
    const auto root = LoadSensorFile(file);
    ValidateKeys(root["remote"], {"freshness_deadline_us"});
    return {ParseFreshnessDeadline(root["remote"]["freshness_deadline_us"],
                                   UINT32_MAX - 1ULL)};
}
}  // namespace dust_control::devices
