/* SPDX-License-Identifier: Apache-2.0 */
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>

#include "../support/usb_report_fixture.hpp"
#include "chassis_input_reader.h"
#include "command_arbiter.hpp"
#include "hardware/motors/motor_encoder.hpp"
#include "joint_output.h"
#include "leg_vmc.h"
#include "robot.hpp"

void Check(bool ok) {
    if (!ok) {
        throw std::runtime_error("Module config contract failed");
    }
}

// 实际计算必须使用启动配置；修改调用方对象不能污染已构造的控制器。
void CheckChassisConfiguration(const modules::ChassisConfig& config) {
    using namespace modules;
    Check(config.remote_deadband == 0.1 && config.max_yaw_rate() == 4.0);
    Check(config.joint_torque_limit == 3.0 && config.leg_length_kp == 900.0);
    auto caller_config = config;
    ChassisInputReader custom({}, caller_config), baseline;
    caller_config.length_gh = 1.0;
    ControllerInputs sources{};
    for (auto& motor : sources.motors) {
        motor.valid = true;
        motor.precise_timestamp_us = 5000;
    }
    sources.imu.valid = true;
    sources.imu.sequence = 1;
    sources.imu.precise_timestamp_us = 5000;
    ChassisCycleInput actual{}, original{};
    custom.Read(sources, actual, 5000, .001, true);
    baseline.Read(sources, original, 5000, .001, true);
    double hx = 0, hz = 0;
    Check(ForwardKinematics(0, 0, 1, hx, hz, config));
    Check(actual.leg.left.hx == hx && actual.leg.left.hz == hz);
    Check(std::abs(actual.leg.left.length - original.leg.left.length) > 1e-4);
    const auto spring =
        ConfiguredGasSpringCompensation(Side::kLeft, actual, config);
    Check(spring.b == 0 && spring.d == 0);
    LegKinematics leg{};
    leg.length = .20;
    leg.jacobian[0][0] = leg.jacobian[1][1] = 1;
    const auto vmc = ComputeLegVmc(leg, .21, 0, 0, 0, 0, 0, config);
    const auto default_vmc = ComputeLegVmc(leg, .21, 0, 0, 0, 0, 0);
    Check(std::abs(vmc.axial_force - 9) < 1e-10);
    Check(std::abs(default_vmc.axial_force - 6) < 1e-10);
    Check(std::abs(
              dust::chassis::ToRemoteAxis(.4, config.max_body_speed, config) -
              .55) < 1e-10);
    dust::chassis::MotionCommandQueue commands;
    dust::chassis::MotionCommand latest;
    Check(commands.Push({.4, 2.0, 0, 1000}));
    ControllerInputs command_input{};
    command_input.remote.robot_enable = true;
    dust::chassis::ApplyMotionCommand(command_input, latest, {true, 10000},
                                      commands, 1001, config);
    Check(std::abs(command_input.remote.chassis_x - .55) < 1e-6);
    Check(std::abs(command_input.remote.chassis_rotate + .55) < 1e-6);
    ChassisControlOutput output{};
    output.joint_control_active = true;
    output.joint_torque = {{100, -100}, {100, -100}};
    actual.feedback_valid = actual.imu_fresh = actual.dm_ready =
        actual.arm_complete = actual.requested_enable = true;
    PrepareJointOutput(ChassisControlState::kBalance, actual, output, config);
    Check(output.joint_torque.left.b == 3 && output.joint_torque.right.d == -3);
    BalanceController balance(chassis_config::kWheelTorqueLimit, config);
    balance.ResetForEnable();
    Check(balance.target_leg_length() == config.stool_target_leg_length);
    ChassisStateMachine custom_state(config), default_state;
    actual.remote_fresh = false;
    actual.imu.pitch_deg = 30;
    custom_state.Update(actual);
    default_state.Update(actual);
    Check(custom_state.tilt_fault_latched() &&
          !default_state.tilt_fault_latched());
}

int main(int argc, char** argv) {
    try {
        using namespace dust::devices;
        // 不同 runtime 配置可独立存在；失败加载不能改动已存在的设备。
        MotorEncoder default_encoder;
        wbr::host::UartReceiveQueue default_store;
        dust::test::UsbReportFixture default_decoder(default_store);
        if (argc == 3) {
            dust::app::Options bad;
            bad.config_directory = argv[2];
            bad.simulate = true;
            bad.usb.realtime = false;
            bool rejected = false;
            try {
                dust::app::Robot invalid(bad);
            } catch (const std::exception&) {
                rejected = true;
            }
            Check(rejected);
        }
        const std::filesystem::path directory(argv[1]);
        const auto chassis = modules::LoadChassisConfig(
            (directory / "chassis_params.yaml").string());
        CheckChassisConfiguration(chassis);
        auto motors =
            LoadMotorConfig((directory / "motors_params.yaml").string());
        const auto imu =
            LoadImuConfig((directory / "sensors_params.yaml").string());
        const auto remote =
            LoadRemoteConfig((directory / "sensors_params.yaml").string());
        Check(motors.wheel_torque_limit_nm() == 2.0);
        Check(motors.wheel_command_sign[0] == 1 &&
              motors.wheel_command_sign[1] == -1);
        Check(motors.dm_range.p_max == 12.56637F &&
              motors.dm_range.t_max == 54);
        const auto address = motors.addresses[0];
        Check(address.feedback_id == 0x202 && address.command_slot == 1);
        wbr::host::UartReceiveQueue store;
        dust::test::UsbReportFixture decoder(store, false, motors, imu);
        MotorEncoder encoder(motors);
        // 构造后修改调用者的配置，不会改变实例路由或编码槽位。
        motors.addresses[0].feedback_id = 0x201;
        motors.addresses[0].command_slot = 0;
        MotorTargets targets{};
        targets[0] = {MotorMode::kTorque, 1.0};
        const auto encoded = encoder.Encode(targets);
        const auto original = default_encoder.Encode(targets);
        Check(encoded && original);
        Check(encoded->frames[0].bytes[0] == 0 &&
              encoded->frames[0].bytes[1] == 0);
        Check(encoded->frames[0].bytes[2] != 0 ||
              encoded->frames[0].bytes[3] != 0);
        Check(encoded->frames[0].bytes[2] == 0x20 &&
              encoded->frames[0].bytes[3] == 0x00);
        targets[0].torque_nm = 1000;
        const auto limited = encoder.Encode(targets);
        Check(limited && limited->frames[0].bytes[2] == 0x40 &&
              limited->frames[0].bytes[3] == 0x00);
        targets = {};
        targets[2] = {MotorMode::kTorque, 1000};
        const auto joint = encoder.Encode(targets);
        Check(joint && (joint->frames[0].bytes[6] & 0x0f) == 0x0f &&
              joint->frames[0].bytes[7] == 0xff);
        Check(original->frames[0].bytes[0] != 0 ||
              original->frames[0].bytes[1] != 0);

        wbr::bridge::UsbInPacket packet{};
        wbr::bridge::CanReceiveReport report{};
        report.header = {10000, 1};
        report.frame_count = 1;
        report.frames[0] = {
            {address.bus, 8, 0, address.feedback_id, {}}, 9000, 0};
        Check(wbr::bridge::Encode(report, packet));
        Check(decoder.Consume(packet, 10000));
        Check(default_decoder.Consume(packet, 10000));
        Check(decoder.Read().control.motors[0].valid);
        Check(!default_decoder.Read().control.motors[0].valid);
        decoder.Reset();
        Check(decoder.Consume(packet, 11000));
        Check(decoder.Read().control.motors[0].valid);

        report.header.transmit_timestamp_us = 12000;
        report.frames[0].frame.id = motors.addresses[2].feedback_id;
        report.frames[0].frame.channel = motors.addresses[2].bus;
        const uint8_t mit_feedback[8] = {0x10, 0xff, 0xff, 0x80, 0, 0, 0, 0};
        std::copy_n(mit_feedback, 8, report.frames[0].frame.data);
        Check(wbr::bridge::Encode(report, packet));
        Check(decoder.Consume(packet, 12000));
        Check(decoder.Read().control.motors[2].position_rad == 12.56637F);
        Check(decoder.Read().control.motors[2].torque_nm == -54);
        Check(default_decoder.Consume(packet, 12000));
        Check(default_decoder.Read().control.motors[2].torque_nm == -54);

        modules::ControllerInputs source{};
        for (auto& motor : source.motors) {
            motor.valid = true;
            motor.precise_timestamp_us = 8000;
        }
        source.imu.valid = true;
        source.imu.sequence = 1;
        source.imu.precise_timestamp_us = 8000;
        source.remote.sequence = 1;
        modules::ChassisInputReader reader(
            {motors.freshness_us, imu.freshness_us, remote.freshness_us});
        modules::ChassisInputReader default_reader;
        modules::ChassisCycleInput input{};
        reader.Read(source, input, 10000, .001, false);
        Check((input.motor_stale_mask & 1) != 0 &&
              (input.motor_stale_mask & 2) == 0);
        Check(!input.imu_fresh);
        default_reader.Read(source, input, 10000, .001, false);
        Check(input.motor_stale_mask == 0 && input.imu_fresh);
        reader.Read(source, input, 11501, .001, false);
        Check(input.remote_fresh);
        reader.Read(source, input, 11502, .001, false);
        Check(!input.remote_fresh);
        // 多个不同配置的 runtime 构造互不排斥，不再有进程级单次加载状态。
        dust::app::Options options;
        options.config_directory = argv[1];
        options.simulate = true;
        options.usb.realtime = false;
        dust::app::Robot first(options);
        dust::app::Robot second(options);
        std::cout << "Module configuration, independent instances, CAN mapping "
                     "and freshness boundaries passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
