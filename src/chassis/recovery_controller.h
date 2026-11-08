/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <cstdint>

#include "chassis_config.h"
#include "chassis_types.h"

namespace dust_control::chassis {

enum class RecoveryPhase : uint8_t {
    kInit = 0U,
    kTurnOver,
    kYawFront,
    kSwing,
    kDrawBack,
    kReturn,
    kFailed,
};

struct RecoveryGimbalStatus {
    bool available = false;
    bool safe_for_leg_motion = true;
    bool aligned = true;
};

struct RecoveryControllerOutput {
    ChassisControlOutput control;
    RecoveryPhase phase = RecoveryPhase::kInit;
    bool request_gimbal_disable = false;
    bool finished = false;
    bool failed = false;
};

class RecoveryController {
   public:
    explicit RecoveryController(ChassisConfig config = {}) : config_(config) {}
    void Reset();
    RecoveryControllerOutput Update(const ChassisCycleInput &input,
                                    RecoveryGimbalStatus gimbal = {});
    RecoveryPhase phase() const {
        return phase_;
    }

   private:
    const ChassisConfig config_;
    enum class SwingEscapePhase : uint8_t {
        kNone = 0U,
        kBackToVertical,
        kToFinal
    };

    struct JointPidState {
        double position_integral = 0.0;
        double speed_integral = 0.0;
    };

    void EnterPhase(RecoveryPhase phase, const ChassisCycleInput &input);
    void UpdatePhase(const ChassisCycleInput &input,
                     const RecoveryGimbalStatus &gimbal, double dt);
    void UpdateTurnOver(const ChassisCycleInput &input,
                        const RecoveryGimbalStatus &gimbal, double dt);
    void UpdateSwing(const ChassisCycleInput &input, double dt);
    void UpdateDrawBack(const ChassisCycleInput &input, double dt);
    bool UpdateJointTargets(const ChassisCycleInput &input);
    double ComputeJointTorque(Side side, Joint joint, double target,
                              double position, double velocity, double dt);
    double RawKinematicAngle(Side side, double normalized_angle);

    RecoveryPhase phase_ = RecoveryPhase::kInit;
    double phase_elapsed_s_ = 0.0;
    double ready_elapsed_s_ = 0.0;
    double upright_elapsed_s_ = 0.0;
    double turnover_stuck_elapsed_s_ = 0.0;
    double turnover_progress_ = 0.0;
    int turnover_direction_ = 1;
    uint32_t turnover_reversals_ = 0U;
    bool reference_initialized_ = false;
    SidePair<double> phase_start_angle_{};
    SidePair<double> leg_length_reference_{};
    SidePair<double> leg_angle_reference_{};
    SidePair<double> swing_stuck_elapsed_s_{};
    SidePair<SwingEscapePhase> swing_escape_phase_{};
    SidePair<bool> target_initialized_{};
    SidePair<JointPair<double>> joint_target_{};
    SidePair<JointPair<JointPidState>> joint_pid_{};
};

}  // namespace dust_control::chassis
