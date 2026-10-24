/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <chassis_params_generated.h>

#include "hardware/motors/dm_protocol.h"

namespace dust::devices {
inline constexpr protocols::DmMitRange kDmRange = {
    .p_min = modules::chassis_params::kDmPositionMin,
    .p_max = modules::chassis_params::kDmPositionMax,
    .v_min = modules::chassis_params::kDmVelocityMin,
    .v_max = modules::chassis_params::kDmVelocityMax,
    .kp_min = modules::chassis_params::kDmKpMin,
    .kp_max = modules::chassis_params::kDmKpMax,
    .kd_min = modules::chassis_params::kDmKdMin,
    .kd_max = modules::chassis_params::kDmKdMax,
    .t_min = modules::chassis_params::kDmTorqueMin,
    .t_max = modules::chassis_params::kDmTorqueMax,
};
}  // namespace dust::devices
