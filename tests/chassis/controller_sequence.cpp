/* SPDX-License-Identifier: Apache-2.0 */
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string_view>

#include "chassis/chassis_controller.h"

namespace chassis = dust_control::chassis;
namespace {
using State = chassis::ChassisControlState;
class Sequence {
   public:
    explicit Sequence(bool trace, chassis::ChassisConfig config = {})
        : trace_(trace),
          config_(config),
          controller_({}, chassis::chassis_config::kWheelTorqueLimit, config) {
        SetPose(0.15);
        sources.imu.valid = true;
        sources.imu.accel_g[2] = sources.imu.vertical_accel_g = 1;
    }
    void SetPose(double length) {
        for (bool right : {false, true}) {
            const auto& hardware = config_.leg_hardware(right);
            bool solved = false;
            double d = 0, b = 0;
            for (int seed_d = -3; seed_d <= 3 && !solved; ++seed_d) {
                for (int seed_b = -3; seed_b <= 3 && !solved; ++seed_b) {
                    solved = chassis::InverseKinematics(
                        length * std::sin(hardware.leg_angle_offset),
                        -length * std::cos(hardware.leg_angle_offset),
                        hardware.kinematic_branch, seed_d, seed_b, d, b,
                        config_);
                }
            }
            assert(solved);
            sources.motors[right ? 4 : 2].position_rad = b;
            sources.motors[right ? 5 : 3].position_rad = d;
        }
    }
    chassis::ChassisControlOutput Tick() {
        const auto now = 1000000ULL + ++ticks * 1000ULL;
        for (auto& motor : sources.motors) {
            motor.valid = feedback_valid;
            motor.enabled = true;
            motor.precise_timestamp_us = now;
        }
        sources.remote.sequence = sources.imu.sequence = ticks;
        sources.imu.precise_timestamp_us = imu_fresh ? now : now - 10000;
        const auto output = controller_.Tick(sources, now, permission);
        if (trace_) {
            std::cout << ticks << ' ' << int(state()) << ' '
                      << output.joint_control_active << ' '
                      << output.bypass_gas_spring_compensation << ' ';
            for (const auto& targets :
                 {output.motor_targets, output.idle_motor_targets}) {
                for (const auto& target : targets) {
                    std::cout << int(target.mode) << ' ' << target.torque_nm
                              << ' ';
                }
            }
            std::cout << output.physical_wheel_torque.left << ' '
                      << output.physical_wheel_torque.right << ' '
                      << output.joint_torque.left.b << ' '
                      << output.joint_torque.left.d << ' '
                      << output.joint_torque.right.b << ' '
                      << output.joint_torque.right.d << '\n';
        }
        return output;
    }
    void EnableAndBalance() {
        Tick();
        Tick();
        sources.remote.robot_enable = true;
        for (unsigned i = 0; i < 300; ++i) {
            Tick();
            assert(state() == State::kDmArming);
        }
        Tick();
        assert(state() == State::kRecovery);
        for (unsigned i = 0; i < 49; ++i) {
            Tick();
            assert(state() == State::kRecovery);
        }
        const auto finished = Tick();
        // 完成周期仍处于回正；下一周期才转换为平衡。
        assert(state() == State::kRecovery && !finished.joint_control_active);
        Tick();
        assert(state() == State::kBalance);
    }
    State state() const {
        return controller_.state();
    }
    chassis::ControllerInputs sources{};
    bool feedback_valid = true, imu_fresh = true, permission = true;
    uint32_t ticks = 0;

   private:
    bool trace_;
    chassis::ChassisConfig config_;
    chassis::ChassisController controller_;
};
void SafetyAndReenable(bool trace) {
    Sequence sequence(trace);
    sequence.EnableAndBalance();
    sequence.feedback_valid = false;
    sequence.Tick();
    assert(sequence.state() == State::kWaitingFeedback);
    sequence.feedback_valid = true;
    sequence.Tick();
    assert(sequence.state() == State::kBalance);
    sequence.imu_fresh = false;
    sequence.Tick();
    assert(sequence.state() == State::kSafetyStop);
    sequence.imu_fresh = true;
    sequence.Tick();
    assert(sequence.state() == State::kDmArming);
    for (unsigned i = 0; i < 300; ++i) {
        sequence.Tick();
    }
    assert(sequence.state() == State::kBalance);
    sequence.sources.imu.pitch_deg = 80;
    sequence.Tick();
    assert(sequence.state() == State::kTiltFault);
    sequence.sources.imu.pitch_deg = 0;
    sequence.Tick();
    assert(sequence.state() == State::kTiltFault);
    sequence.sources.remote.robot_enable = false;
    sequence.Tick();
    assert(sequence.state() == State::kDisabled);
    sequence.sources.remote.robot_enable = true;
    sequence.permission = false;
    sequence.Tick();
    assert(sequence.state() == State::kDisabled);
}
void ActionFailure(bool trace, bool jump) {
    Sequence sequence(trace);
    sequence.EnableAndBalance();
    sequence.sources.remote.jump = jump;
    sequence.sources.remote.climb_stairs = !jump;
    const auto action = jump ? State::kJump : State::kClimbStairs;
    const auto started = sequence.Tick();
    assert(sequence.state() == action && started.joint_control_active);
    bool finished = false;
    for (unsigned i = 0; i < 2000; ++i) {
        const auto output = sequence.Tick();
        if (!output.joint_control_active) {
            assert(sequence.state() == action);
            finished = true;
            break;
        }
    }
    assert(finished);
    sequence.Tick();
    assert(sequence.state() == State::kActionFault);
    sequence.Tick();
    assert(sequence.state() == State::kActionFault);
    sequence.sources.remote.robot_enable = false;
    sequence.Tick();
    assert(sequence.state() == State::kDisabled);
}
void RecoveryFailure(bool trace) {
    Sequence sequence(trace);
    sequence.sources.remote.robot_enable = true;
    for (unsigned i = 0; i < 300; ++i) {
        sequence.Tick();
    }
    sequence.sources.imu.vertical_accel_g = 0;
    const auto failed = sequence.Tick();
    assert(sequence.state() == State::kRecovery &&
           !failed.joint_control_active);
    sequence.Tick();
    assert(sequence.state() == State::kActionFault);
}
void FlightFailure(bool trace) {
    chassis::ChassisConfig config;
    config.gas_spring_left_force = config.gas_spring_right_force = 0;
    Sequence sequence(trace, config);
    sequence.EnableAndBalance();
    sequence.SetPose(0.20);
    bool airborne = false;
    for (unsigned i = 0; i < 500; ++i) {
        sequence.Tick();
        if (sequence.state() == State::kFlight) {
            airborne = true;
            break;
        }
    }
    assert(airborne);
    bool failed = false;
    for (unsigned i = 0; i < 3000; ++i) {
        const auto output = sequence.Tick();
        if (!output.joint_control_active) {
            assert(sequence.state() == State::kFlight);
            failed = true;
            break;
        }
    }
    assert(failed);
    sequence.Tick();
    assert(sequence.state() == State::kActionFault);
}
}  // namespace
int main(int argc, char** argv) {
    const bool trace = argc == 2 && std::string_view(argv[1]) == "--trace";
    std::cout << std::setprecision(17);
    SafetyAndReenable(trace);
    ActionFailure(trace, true);
    ActionFailure(trace, false);
    RecoveryFailure(trace);
    FlightFailure(trace);
}
