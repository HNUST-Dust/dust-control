/* SPDX-License-Identifier: Apache-2.0 */
#include <cassert>
#include <iostream>
#include <string_view>

#include "chassis/chassis_state_machine.h"

namespace chassis = dust_control::chassis;
namespace {
using State = chassis::ChassisControlState;
using Reason = chassis::ChassisTransitionReason;
struct Scenario {
    explicit Scenario(bool trace) : trace(trace) {
        input.requested_enable = input.remote.robot_enable = true;
        input.remote_fresh = input.imu_fresh = input.feedback_valid = true;
        input.arm_complete = input.dm_ready = true;
    }
    chassis::ChassisStateTransition Tick() {
        const auto result = machine.Update(input);
        assert(result.to == machine.state());
        assert(result.changed == (result.from != result.to));
        if (trace) {
            std::cout << int(result.from) << ' ' << int(result.to) << ' '
                      << int(result.reason) << ' ' << result.changed << ' '
                      << result.enable_changed << ' ' << result.enabled << ' '
                      << machine.tilt_fault_latched() << ' '
                      << int(machine.arm_reset_reason()) << '\n';
        }
        return result;
    }
    void Expect(State state, Reason reason = Reason::kNone) {
        const auto result = Tick();
        assert(result.to == state && result.reason == reason);
    }
    void Balance() {
        Expect(State::kDmArming, Reason::kRemoteEnabled);
        Expect(State::kRecovery, Reason::kRecoveryStarted);
        machine.MarkRecoveryCompleted();
        Expect(State::kBalance, Reason::kRecoveryCompleted);
    }
    void Jump() {
        input.remote.jump = true;
        Expect(State::kJump, Reason::kJumpRequested);
    }
    chassis::ChassisStateMachine machine;
    chassis::ChassisCycleInput input{};
    bool trace;
};

void Priorities(bool trace) {
    Scenario flight(trace);
    flight.Balance();
    flight.input.remote.jump = flight.input.remote.climb_stairs = true;
    flight.machine.SetFlightDetected(true);
    flight.Expect(State::kFlight, Reason::kFlightDetected);
    flight.machine.SetFlightDetected(false);
    flight.machine.MarkFlightCompleted();
    flight.Expect(State::kBalance, Reason::kFlightLanded);
    flight.Expect(State::kBalance);  // 被离地检测挡住的请求不会排队。

    Scenario jump(trace);
    jump.Balance();
    jump.input.remote.jump = jump.input.remote.climb_stairs = true;
    jump.Expect(State::kJump, Reason::kJumpRequested);
    jump.machine.MarkJumpCompleted();
    jump.Expect(State::kBalance, Reason::kJumpCompleted);
    jump.Expect(State::kBalance);  // 持续按住不能再次触发。
    jump.input.remote.jump = jump.input.remote.climb_stairs = false;
    jump.Tick();
    jump.input.remote.jump = jump.input.remote.climb_stairs = true;
    jump.Expect(State::kClimbStairs,
                Reason::kClimbRequested);  // 跳跃仍在冷却。

    Scenario readiness(trace);
    readiness.Balance();
    readiness.input.arm_complete = readiness.input.feedback_valid = false;
    readiness.input.remote.jump = true;
    readiness.Expect(State::kDmArming, Reason::kDmNotReady);
    readiness.input.arm_complete = true;
    readiness.Expect(State::kWaitingFeedback, Reason::kArmingCompleted);
    readiness.input.feedback_valid = true;
    readiness.Expect(State::kBalance, Reason::kFeedbackRecovered);
    readiness.Expect(State::kBalance);

    Scenario faults(trace);
    faults.Balance();
    faults.input.imu.pitch_deg = 80;
    faults.Expect(State::kTiltFault, Reason::kTiltExceeded);
    faults.input.imu_fresh = false;
    faults.Expect(State::kSafetyStop, Reason::kImuLost);
    faults.machine.MarkJumpFailed();
    faults.Expect(State::kSafetyStop,
                  Reason::kImuLost);  // 本周期末才锁存动作失败。
    faults.Expect(State::kActionFault, Reason::kJumpFailed);
    faults.input.requested_enable = false;
    faults.input.remote_fresh = false;
    faults.Expect(State::kDisabled, Reason::kRemoteTimeout);
    faults.input.requested_enable = faults.input.imu_fresh = true;
    faults.input.imu.pitch_deg = 0;
    faults.Expect(State::kActionFault,
                  Reason::kJumpFailed);  // 超时不能清故障。
    faults.input.requested_enable = false;
    faults.input.remote.robot_enable = false;
    faults.input.remote_fresh = true;
    faults.Expect(State::kDisabled, Reason::kRemoteDisabled);
    assert(!faults.machine.tilt_fault_latched());
    faults.input.requested_enable = faults.input.remote.robot_enable = true;
    faults.Expect(State::kDmArming, Reason::kRemoteEnabled);
    faults.Expect(State::kRecovery, Reason::kRecoveryStarted);
}

void FeedbackTiming(bool trace) {
    Scenario failure(trace);
    failure.Balance();
    failure.Jump();
    failure.machine.MarkJumpCompleted();
    failure.machine.MarkJumpFailed();
    failure.input.feedback_valid = false;
    failure.Expect(State::kWaitingFeedback, Reason::kFeedbackLost);
    failure.Expect(State::kActionFault, Reason::kJumpFailed);

    Scenario completion(trace);
    completion.Balance();
    completion.Jump();
    completion.machine.MarkJumpCompleted();
    completion.Expect(State::kBalance, Reason::kJumpCompleted);
    completion.input.dt = 1.0;
    for (unsigned i = 0; i < 3; ++i) {
        completion.Expect(State::kBalance);  // 冷却结束也不重放被按住的请求。
    }
    completion.input.remote.jump = false;
    completion.Tick();
    completion.Jump();
    completion.Expect(State::kJump);  // 重新进入时清除上次完成反馈。
    completion.machine.MarkJumpFailed();
    completion.Expect(State::kActionFault, Reason::kJumpFailed);

    Scenario disabled(trace);
    disabled.input.requested_enable = disabled.input.remote.robot_enable =
        false;
    disabled.input.remote.jump = disabled.input.remote.climb_stairs = true;
    disabled.Expect(State::kDisabled, Reason::kRemoteDisabled);
    disabled.input.requested_enable = disabled.input.remote.robot_enable = true;
    disabled.Balance();
    disabled.Expect(State::kBalance);  // 禁用期间的请求边沿也被消费。
}

Scenario AtState(State state, bool trace) {
    Scenario scenario(trace);
    if (state == State::kDisabled) {
        return scenario;
    }
    scenario.Expect(State::kDmArming, Reason::kRemoteEnabled);
    if (state == State::kDmArming) {
        return scenario;
    }
    scenario.Expect(State::kRecovery, Reason::kRecoveryStarted);
    if (state == State::kRecovery) {
        return scenario;
    }
    scenario.machine.MarkRecoveryCompleted();
    scenario.Expect(State::kBalance, Reason::kRecoveryCompleted);
    switch (state) {
        case State::kBalance:
            break;
        case State::kFlight:
            scenario.machine.SetFlightDetected(true);
            scenario.Expect(state, Reason::kFlightDetected);
            break;
        case State::kJump:
            scenario.Jump();
            break;
        case State::kClimbStairs:
            scenario.input.remote.climb_stairs = true;
            scenario.Expect(state, Reason::kClimbRequested);
            break;
        case State::kWaitingFeedback:
            scenario.input.feedback_valid = false;
            scenario.Expect(state, Reason::kFeedbackLost);
            break;
        case State::kSafetyStop:
            scenario.input.imu_fresh = false;
            scenario.Expect(state, Reason::kImuLost);
            break;
        case State::kTiltFault:
            scenario.input.imu.pitch_deg = 80;
            scenario.Expect(state, Reason::kTiltExceeded);
            break;
        case State::kActionFault:
            scenario.Jump();
            scenario.machine.MarkJumpFailed();
            scenario.Expect(state, Reason::kJumpFailed);
            break;
        default:
            assert(false);
    }
    return scenario;
}

void FailurePriorities(bool trace) {
    for (auto state : {State::kRecovery, State::kFlight, State::kJump,
                       State::kClimbStairs}) {
        auto scenario = AtState(state, trace);
        Reason reason = Reason::kNone;
        switch (state) {
            case State::kRecovery:
                scenario.machine.MarkRecoveryCompleted();
                scenario.machine.MarkRecoveryFailed();
                reason = Reason::kRecoveryFailed;
                break;
            case State::kFlight:
                scenario.machine.MarkFlightCompleted();
                scenario.machine.MarkFlightFailed();
                reason = Reason::kFlightFailed;
                break;
            case State::kJump:
                scenario.machine.MarkJumpCompleted();
                scenario.machine.MarkJumpFailed();
                reason = Reason::kJumpFailed;
                break;
            case State::kClimbStairs:
                scenario.machine.MarkClimbCompleted();
                scenario.machine.MarkClimbFailed();
                reason = Reason::kClimbFailed;
                break;
            default:
                assert(false);
        }
        scenario.Expect(State::kActionFault, reason);
        scenario.Expect(State::kActionFault, reason);
    }
    for (unsigned mask = 1; mask < 16; ++mask) {
        Scenario scenario(trace);
        scenario.Balance();
        if (mask & 1) {
            scenario.machine.MarkRecoveryFailed();
        }
        if (mask & 2) {
            scenario.machine.MarkFlightFailed();
        }
        if (mask & 4) {
            scenario.machine.MarkJumpFailed();
        }
        if (mask & 8) {
            scenario.machine.MarkClimbFailed();
        }
        // 非当前动作的失败也会在本周期末锁存，下一周期按固定原因优先级报告。
        scenario.Expect(State::kBalance);
        const Reason reason = mask & 1   ? Reason::kRecoveryFailed
                              : mask & 2 ? Reason::kFlightFailed
                              : mask & 4 ? Reason::kJumpFailed
                                         : Reason::kClimbFailed;
        scenario.Expect(State::kActionFault, reason);
    }
}

void ConditionCombinations(bool trace) {
    for (auto state :
         {State::kDisabled, State::kSafetyStop, State::kDmArming,
          State::kWaitingFeedback, State::kRecovery, State::kBalance,
          State::kFlight, State::kJump, State::kClimbStairs,
          State::kActionFault, State::kTiltFault}) {
        const auto baseline = AtState(state, trace);
        for (unsigned mask = 0; mask < 1024; ++mask) {
            auto scenario = baseline;
            scenario.input.requested_enable = mask & 1;
            scenario.input.remote.robot_enable =
                scenario.input.requested_enable;
            scenario.input.remote_fresh = mask & 2;
            scenario.input.imu_fresh = mask & 4;
            scenario.input.arm_complete = mask & 8;
            scenario.input.dm_ready = mask & 16;
            scenario.input.feedback_valid = mask & 32;
            scenario.input.imu.pitch_deg = mask & 64 ? 80 : 0;
            scenario.input.remote.jump = mask & 128;
            scenario.input.remote.climb_stairs = mask & 256;
            scenario.machine.SetFlightDetected(mask & 512);
            scenario.Tick();
            scenario.Tick();  // 同时比较锁存及边沿影响的下一周期。
        }
        for (unsigned mask = 0; mask < 16; ++mask) {
            auto scenario = baseline;
            if (mask & 1) {
                scenario.machine.MarkRecoveryFailed();
                scenario.machine.MarkRecoveryCompleted();
            }
            if (mask & 2) {
                scenario.machine.MarkFlightFailed();
                scenario.machine.MarkFlightCompleted();
            }
            if (mask & 4) {
                scenario.machine.MarkJumpFailed();
                scenario.machine.MarkJumpCompleted();
            }
            if (mask & 8) {
                scenario.machine.MarkClimbFailed();
                scenario.machine.MarkClimbCompleted();
            }
            scenario.Tick();
            scenario.Tick();
        }
    }
}
}  // namespace
int main(int argc, char** argv) {
    const bool trace = argc == 2 && std::string_view(argv[1]) == "--trace";
    Priorities(trace);
    FeedbackTiming(trace);
    FailurePriorities(trace);
    ConditionCombinations(trace);
}
