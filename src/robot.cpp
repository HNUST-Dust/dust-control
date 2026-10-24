/* SPDX-License-Identifier: Apache-2.0 */
#include "robot.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace dust::app {
Robot::Robot(const Options& options) : options_(options) {
    ValidateOptions(options_);
    if (!options_.config_directory.empty()) {
        const std::filesystem::path directory(options_.config_directory);
        const auto motors = devices::LoadMotorConfig(
            (directory / "motors_params.yaml").string());
        const auto imu = devices::LoadImuConfig(
            (directory / "sensors_params.yaml").string());
        const auto remote = devices::LoadRemoteConfig(
            (directory / "sensors_params.yaml").string());
        motors_ = motors;
        imu_ = imu;
        remote_ = remote;
        std::cout << "Configuration loaded: " << options_.config_directory
                  << std::endl;
    }
}
Robot::~Robot() {
    Stop();
}
void Robot::Start() {
    if (started_ || stopped_) {
        throw std::logic_error("Robot cannot be restarted");
    }
    started_ = true;
    try {
        if (!options_.simulate) {
            usb_ = std::make_unique<wbr::host::UsbSession>(options_.usb,
                                                           motors_, imu_);
        }
        if (options_.telemetry_port || options_.foxglove_port) {
            telemetry_ = std::make_unique<wbr::host::TelemetryPublisher>(
                options_.telemetry_port, options_.control_cpu, options_.usb.cpu,
                options_.foxglove_port);
        }
        chassis::ChassisRuntimeOptions control;
        control.realtime = options_.usb.realtime;
        control.cpu = options_.control_cpu;
        control.priority = options_.usb.priority;
        control.enable_output = options_.enable_output;
        control.seconds = options_.seconds;
        control.commands = {options_.ros_commands, options_.command_timeout_ns};
        chassis::ChassisDevices devices;
        if (usb_) {
            const auto& link = usb_->link();
            devices = {&link.motor_decoder(), &link.sbus_decoder(),
                       &link.hi91_decoder(), &usb_->transmitter()};
        }
        chassis_ = std::make_unique<chassis::ChassisRuntime>(
            control, motors_,
            modules::InputDeadlines{motors_.freshness_us, imu_.freshness_us,
                                    remote_.freshness_us},
            devices, telemetry_.get());
        if (usb_) {
            if (!usb_->Start()) {
                throw std::runtime_error(usb_->error());
            }
            std::cout << "Bridge capabilities=0x" << std::hex
                      << usb_->link().capabilities() << std::dec
                      << "; control IMU=HI91" << std::endl;
        }
        chassis_->Start();
    } catch (...) {
        Stop();
        throw;
    }
}
void Robot::Stop() {
    // 先回收全部业务线程，再停止 USB 发送和接收，最后释放遥测资源。
    if (chassis_) {
        chassis_->Stop();
    }
    if (usb_) {
        usb_->Stop();
    }
    telemetry_.reset();
    stopped_ = true;
}
bool Robot::finished() const {
    return stopped_ || (chassis_ && chassis_->finished()) ||
           (usb_ && !usb_->healthy());
}
std::string Robot::error() const {
    if (chassis_ && chassis_->finished() && !chassis_->error().empty()) {
        return chassis_->error();
    }
    return usb_ ? usb_->error() : std::string{};
}
chassis::ChassisRuntime& Robot::chassis() {
    if (!chassis_) {
        throw std::logic_error("Robot modules have not been started");
    }
    return *chassis_;
}
}  // namespace dust::app
