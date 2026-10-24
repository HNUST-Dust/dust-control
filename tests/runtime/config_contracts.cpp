/* SPDX-License-Identifier: Apache-2.0 */
#include <filesystem>
#include <iostream>
#include <stdexcept>

#include "../support/usb_report_fixture.hpp"
#include "chassis_input_reader.h"
#include "hardware/motors/motor_encoder.hpp"
#include "robot.hpp"

void Check(bool ok) {
    if (!ok) {
        throw std::runtime_error("Module config contract failed");
    }
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
        auto motors =
            LoadMotorConfig((directory / "motors_params.yaml").string());
        const auto imu =
            LoadImuConfig((directory / "sensors_params.yaml").string());
        const auto remote =
            LoadRemoteConfig((directory / "sensors_params.yaml").string());
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
