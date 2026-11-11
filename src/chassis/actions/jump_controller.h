/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <optional>

#include "chassis_config.h"
#include "chassis_types.h"
#include "chassis/actions/flight_controller.h"

namespace dust_control::chassis {
enum class JumpPhase : uint8_t {
    kInit = 0U,
    kPreCompress,
    kThrust,
    kAirRetract,
    kAir,
    kReturn,
    kFailed
};

struct JumpControllerOutput {
    ChassisControlOutput control;
    // 有值时由总控运行平衡预压缩；control 在该周期不提供动作输出。
    std::optional<double> precompress_leg_length;
    // 阶段处理后的状态，不能据此推断本周期输出所属阶段。
    JumpPhase phase = JumpPhase::kInit;
    bool finished = false;
    bool failed = false;
};

class JumpController {
   public:
    explicit JumpController(ChassisConfig config = {})
        : config_(config), airborne_controller_(config) {}
    void Reset();
    JumpControllerOutput Update(const ChassisCycleInput &input);
    JumpPhase phase() const {
        return phase_;
    }

   private:
    const ChassisConfig config_;
    void EnterPhase(JumpPhase phase, const ChassisCycleInput &input);
    void UpdatePhase(const ChassisCycleInput &input, double dt,
                     JumpControllerOutput &output);
    void UpdatePreCompress(const ChassisCycleInput &input,
                           JumpControllerOutput &output);
    void UpdateThrust(const ChassisCycleInput &input,
                      ChassisControlOutput &output);
    void UpdateAirRetract(const ChassisCycleInput &input, double dt,
                          ChassisControlOutput &output);
    void ComputeAttitudeControl(const ChassisCycleInput &input,
                                double actuator[4]) const;
    void ComputeVmcLeg(const ChassisCycleInput &input, Side side,
                       double target_length, double target_rate,
                       double support_force, double leg_torque,
                       ChassisControlOutput &output) const;
    JumpPhase phase_ = JumpPhase::kInit;
    double phase_elapsed_s_ = 0.0;
    double total_elapsed_s_ = 0.0;
    double air_retract_hold_s_ = 0.0;
    double yaw_reference_ = 0.0;
    SidePair<double> theta_reference_;
    SidePair<double> retract_reference_;
    FlightController airborne_controller_;
};
}  // namespace dust_control::chassis
