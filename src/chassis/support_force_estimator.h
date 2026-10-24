/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "chassis_types.h"

namespace modules {

/** 根据关节反馈力矩与 IMU 估计左右轮的垂直地面反力。 */
class SupportForceEstimator {
   public:
    void Reset();
    ChassisSupportForceEstimate Update(const ChassisCycleInput &input);

   private:
    struct SideState {
        double previous_length_rate = 0.0;
        double previous_theta_rate = 0.0;
        double length_accel = 0.0;
        double theta_accel = 0.0;
        double filtered_force = 0.0;
    };

    SidePair<SideState> side_state_;
    bool initialized_ = false;
};

}  // namespace modules
