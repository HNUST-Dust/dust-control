/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "chassis/balance/balance_controller.h"
#include "chassis_config.h"
#include "chassis/control/chassis_input_reader.h"
#include "chassis/control/chassis_output_policy.h"
#include "chassis/control/chassis_state_machine.h"
#include "chassis/actions/climb_stairs_controller.h"
#include "chassis/actions/flight_controller.h"
#include "chassis/actions/jump_controller.h"
#include "chassis/actions/recovery_controller.h"
#include "chassis/estimation/support_force_estimator.h"

namespace dust_control::chassis {
// 控制周期的同步执行入口，不依赖线程、硬件传输、全局通道或遥测。
class ChassisController {
   public:
    explicit ChassisController(
        InputDeadlines deadlines = {},
        double wheel_torque_limit = chassis_config::kWheelTorqueLimit,
        ChassisConfig config = {});
    // 按需向调用方同步复制本周期输入，控制核心内不执行遥测 I/O。
    ChassisControlOutput Tick(const ControllerInputs &, uint64_t now_us,
                              bool operator_permission = false,
                              ChassisCycleInput *observed_input = nullptr);
    ChassisControlState state() const {
        return state_machine_.state();
    }

   private:
    void UpdateControlState(const ChassisCycleInput &);
    void ComputeControlOutput(const ChassisCycleInput &,
                              ChassisControlOutput &);
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
