/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "chassis_types.h"

namespace modules {

/** 控制器内部的输出策略：将状态与物理控制量转换为本周期执行器目标。 */
class ChassisOutputPolicy {
   public:
    /** 按状态执行输出策略；返回 true 表示调用方须复位控制状态。 */
    bool Apply(ChassisControlState state, uint32_t loop_ticks,
               const ChassisCycleInput &input, ChassisControlOutput &output);
    bool arm_complete() const;
    void ResetArming() {
        dm_arm_ticks_ = 0U;
    }

   private:
    uint32_t dm_arm_ticks_ = 0U;
};

}  // namespace modules
