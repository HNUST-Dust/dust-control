/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "chassis/controller_inputs.h"
#include "chassis_config.h"
#include "chassis_types.h"

namespace dust_control::chassis {

struct InputDeadlines {
    std::array<uint64_t, 6> motors{3000, 3000, 3000, 3000, 3000, 3000};
    uint64_t hi91_us = 3000;
    uint64_t remote_us = 1000000;
};

/** 保存新鲜度历史，将设备物理反馈转换为本周期运动学状态。 */
class ChassisInputReader {
   public:
    explicit ChassisInputReader(InputDeadlines deadlines = {},
                                ChassisConfig config = {})
        : config_(config), deadlines_(deadlines) {}
    void Read(const ControllerInputs &sources, ChassisCycleInput &input,
              uint64_t now_us, double dt, bool arm_complete);

   private:
    const ChassisConfig config_;
    const InputDeadlines deadlines_;
    uint32_t last_remote_sequence_ = 0U;
    uint64_t last_remote_update_us_ = 0U;
    uint32_t last_imu_sequence_ = 0U;
};

}  // namespace dust_control::chassis
