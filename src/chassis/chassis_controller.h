/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "balance_controller.h"
#include "chassis_config.h"
#include "chassis_input_reader.h"
#include "chassis_output_policy.h"
#include "chassis_state_machine.h"
#include "climb_stairs_controller.h"
#include "flight_controller.h"
#include "jump_controller.h"
#include "recovery_controller.h"
#include "support_force_estimator.h"

namespace dust_control::chassis {
// 控制周期的同步执行入口，不依赖线程、USB、RTOS、全局通道或遥测。
class ChassisController {
   public:
    explicit ChassisController(
        InputDeadlines deadlines = {},
        double wheel_torque_limit = chassis_config::kWheelTorqueLimit,
        ChassisConfig config = {})
        : config_(config),
          recovery_controller_(config),
          climb_stairs_controller_(config),
          balance_controller_(wheel_torque_limit, config),
          input_reader_(deadlines, config),
          support_force_estimator_(config),
          flight_controller_(config),
          jump_controller_(config),
          state_machine_(config),
          output_policy_(config) {
        ValidateChassisConfig(config_);
    }
    // 按需向调用方同步复制本周期输入，控制核心内不执行遥测 I/O。
    ChassisControlOutput Tick(const ControllerInputs &, uint64_t now_us,
                              bool operator_permission = false,
                              ChassisCycleInput *observed_input = nullptr);
    ChassisControlState state() const {
        return state_machine_.state();
    }

   private:
    const ChassisConfig config_;
    void UpdateControlState(const ChassisCycleInput &);
    void ComputeControlOutput(const ChassisCycleInput &,
                              const ChassisSupportForceEstimate &,
                              ChassisControlOutput &);
    void ApplyControlOutput(const ChassisCycleInput &, ChassisControlOutput &);
    void ResetControlState();
    uint64_t last_us_ = 0;
    uint32_t loop_ticks_ = 0;
    RecoveryController recovery_controller_;
    ClimbStairsController climb_stairs_controller_;
    BalanceController balance_controller_;
    ChassisInputReader input_reader_;
    SupportForceEstimator support_force_estimator_;
    FlightController flight_controller_;
    JumpController jump_controller_;
    ChassisStateMachine state_machine_;
    ChassisOutputPolicy output_policy_;
};
}  // namespace dust_control::chassis
