/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <cstdint>

#include "chassis_config.h"
#include "chassis_types.h"

namespace dust_control::chassis {

enum class ClimbStairsPhase : uint8_t {
    kInit = 0U,
    kMotion1,
    kMotion2,
    kReturn,
    kFailed,
};

struct ClimbStairsControllerOutput {
    ChassisControlOutput control;
    ClimbStairsPhase phase = ClimbStairsPhase::kInit;
    bool finished = false;
    bool failed = false;
};

/** 连续执行 Init → Motion1 → Motion2 → Return 爬楼动作。 */
class ClimbStairsController {
   public:
    explicit ClimbStairsController(ChassisConfig config = {})
        : config_(config) {}
    void Reset();
    ClimbStairsControllerOutput Update(const ChassisCycleInput &input);
    ClimbStairsPhase phase() const {
        return phase_;
    }

   private:
    const ChassisConfig config_;
    struct JointPidState {
        double position_integral = 0.0;
        double speed_integral = 0.0;
    };

    void EnterPhase(ClimbStairsPhase phase);
    void UpdatePhase(const ChassisCycleInput &input, double dt);
    void UpdateMotion1(const ChassisCycleInput &input, double dt);
    void UpdateMotion2(const ChassisCycleInput &input, double dt);
    bool UpdateJointTargets(const ChassisCycleInput &input);
    double ComputeJointTorque(Side side, Joint joint, double target,
                              double position, double velocity, double dt);
    double RawKinematicAngle(Side side, double normalized_angle);

    ClimbStairsPhase phase_ = ClimbStairsPhase::kInit;
    double phase_elapsed_s_ = 0.0;
    double ready_elapsed_s_ = 0.0;
    bool reference_initialized_ = false;
    SidePair<bool> target_initialized_{};
    SidePair<double> leg_length_reference_{};
    SidePair<double> leg_angle_reference_{};
    SidePair<JointPair<double>> joint_target_{};
    SidePair<JointPair<JointPidState>> joint_pid_{};
};

}  // namespace dust_control::chassis
