/* SPDX-License-Identifier: Apache-2.0 */
#include "robot.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace dust::app {
Robot::Robot(const std::string& config_directory)
    : Robot(LoadRobotConfig(
                (std::filesystem::path(config_directory) / "robot.yaml")
                    .string()),
            config_directory) {}
Robot::Robot(RobotConfig config, const std::string& config_directory)
    : config_(std::move(config)) {
    ValidateRobotConfig(config_);
    const std::filesystem::path directory(config_directory);
    if (!directory.empty()) {
        if (config_.chassis.enabled) {
            chassis_config_ = modules::LoadChassisConfig(
                (directory / "chassis_params.yaml").string());
            remote_ = devices::LoadRemoteConfig(
                (directory / "sensors_params.yaml").string());
        }
        imu_ = devices::LoadImuConfig(
            (directory / "sensors_params.yaml").string());
    }
    // 配置先完整加载；此处不打开 USB，也不启动线程。
    for (auto& [name, adapter] : config_.adapters) {
        adapter.usb.thread_name = "usb-" + name + "-rx";
        adapter.transmit.thread_name = "usb-" + name + "-tx";
    }
    if (config_.chassis.enabled) {
        chassis_motors_ = devices::LoadMotorConfig(
            (directory / config_.motors_file).string());
        if (!std::all_of(chassis_motors_.present.begin(),
                         chassis_motors_.present.end(),
                         [](bool value) { return value; })) {
            throw std::invalid_argument(
                "Chassis requires all six named motors");
        }
        for (const auto& owner : chassis_motors_.adapters) {
            if (!config_.adapters.contains(owner)) {
                throw std::invalid_argument("Unknown motor adapter: " + owner);
            }
            if (!motor_configs_.contains(owner)) {
                motor_configs_.emplace(owner, devices::MotorConfigForAdapter(
                                                  chassis_motors_, owner));
            }
        }
        if (!motor_configs_.contains(config_.chassis.adapter)) {
            throw std::invalid_argument(
                "Chassis sensor adapter requires a real CAN producer");
        }
    }
    if (!directory.empty()) {
        std::cout << "Configuration loaded: " << directory.string()
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
        CreateAdapters();
        CreateChassis();
        CreateGimbal();
        CreateUi();
        // 先启动底盘；每板就绪后立即收到安全输出，全部成功前禁止使能。
        if (chassis_) {
            chassis_->Start();
        }
        StartAdapters();
        StartModulesFor("");
        if (finished()) {
            throw std::runtime_error("Module startup failed: " + error());
        }
        startup_ready_ = true;
    } catch (...) {
        Stop();
        throw;
    }
}
void Robot::CreateAdapters() {
    if (!config_.execution.simulate) {
        for (const auto& [name, adapter] : config_.adapters) {
            const bool motor_source =
                config_.chassis.enabled && motor_configs_.contains(name);
            const bool used =
                motor_source ||
                (config_.chassis.enabled && config_.chassis.adapter == name) ||
                (config_.gimbal.enabled && config_.gimbal.adapter == name) ||
                (config_.ui.enabled && config_.ui.adapter == name);
            if (!used) {
                continue;
            }
            auto usb = adapter.usb;
            auto transmit = adapter.transmit;
            usb.realtime = transmit.realtime = config_.execution.realtime;
            const auto motors = motor_configs_.find(name);
            usb.decode_motors = motors != motor_configs_.end();
            auto session = std::make_unique<wbr::host::UsbSession>(
                usb,
                usb.decode_motors ? motors->second : devices::MotorConfig{},
                imu_, transmit);
            adapters_.try_emplace(name, std::move(session));
        }
    }
}

void Robot::CreateChassis() {
    if (config_.chassis.enabled) {
        const auto& motors = chassis_motors_;
        chassis::ChassisRuntimeOptions control;
        control.thread = config_.chassis.thread;
        control.thread.realtime = config_.execution.realtime;
        control.enable_output = config_.execution.enable_output;
        control.startup_ready = &startup_ready_;
        chassis::ChassisDevices devices;
        if (auto* usb = SessionFor(config_.chassis.adapter)) {
            devices.remote = &usb->link().sbus_decoder();
            devices.imu = &usb->link().hi91_decoder();
        }
        for (const auto& [name, config] : motor_configs_) {
            if (auto* usb = SessionFor(name)) {
                if (std::any_of(config.present.begin(), config.present.end(),
                                [](bool value) { return value; })) {
                    devices.motor_adapters.push_back(
                        {name, &usb->link().motor_decoder(),
                         &usb->transmitter(), config,
                         &adapters_.at(name).started});
                }
            }
        }
        chassis_ = std::make_unique<chassis::ChassisRuntime>(
            control, motors,
            modules::InputDeadlines{motors.freshness_us, imu_.freshness_us,
                                    remote_.freshness_us},
            devices, chassis_config_);
    }
}

void Robot::CreateGimbal() {
    if (config_.gimbal.enabled) {
        gimbal::GimbalDevices devices;
        if (auto* usb = SessionFor(config_.gimbal.adapter)) {
            if (motor_configs_.contains(config_.gimbal.adapter)) {
                devices.motors = &usb->link().motor_decoder();
            }
            devices.imu = &usb->link().hi91_decoder();
            devices.can.emplace(usb->transmitter(),
                                wbr::host::CanProducer::kGimbal);
        }
        auto thread = config_.gimbal.thread;
        thread.realtime = config_.execution.realtime;
        gimbal_ = std::make_unique<gimbal::GimbalRuntime>(devices, thread);
    }
}

void Robot::CreateUi() {
    if (config_.ui.enabled) {
        ui::UiDevices devices;
        if (auto* usb = SessionFor(config_.ui.adapter)) {
            devices.referee = &usb->link().referee_decoder();
            devices.uart.emplace(usb->transmitter());
        }
        auto thread = config_.ui.thread;
        thread.realtime = config_.execution.realtime;
        ui_ = std::make_unique<ui::UiRuntime>(devices, thread);
    }
}

void Robot::StartAdapters() {
    // 每块板就绪后立即启动绑定的模块，避免等待其他板时耗尽本板租约。
    // 输出仍受 startup_ready_ 限制；任一后续启动失败均回收全部会话。
    for (const auto& [name, config] : config_.adapters) {
        const auto found = adapters_.find(name);
        if (found != adapters_.end()) {
            auto& adapter = found->second;
            if (adapter.session->transmitter().HasCanProducers()) {
                if (!adapter.session->Start()) {
                    throw std::runtime_error("Adapter " + name + ": " +
                                             adapter.session->error());
                }
                adapter.started = true;
                std::cout << "Adapter " << name << ": Bridge capabilities=0x"
                          << std::hex << adapter.session->link().capabilities()
                          << std::dec << std::endl;
            } else {
                std::cout << "Adapter " << name
                          << ": configured, idle (no CAN producer)"
                          << std::endl;
            }
        }
        StartModulesFor(name);
    }
}
void Robot::StartModulesFor(const std::string& adapter) {
    if (gimbal_ && config_.gimbal.adapter == adapter) {
        gimbal_->Start();
    }
    if (ui_ && config_.ui.adapter == adapter) {
        ui_->Start();
    }
}

wbr::host::UsbSession* Robot::SessionFor(const std::string& name) {
    const auto found = adapters_.find(name);
    return found == adapters_.end() ? nullptr : found->second.session.get();
}

void Robot::Stop() {
    startup_ready_ = false;
    // 所有借用者先退出，之后再回收各板的发送与事件线程。
    if (chassis_) {
        chassis_->Stop();
    }
    if (gimbal_) {
        gimbal_->Stop();
    }
    if (ui_) {
        ui_->Stop();
    }
    // 各发送线程并发提交 Stop；单板的保护等待不能消耗其他板的租约。
    for (auto& [name, adapter] : adapters_) {
        adapter.session->transmitter().RequestStop();
    }
    for (auto& [name, adapter] : adapters_) {
        adapter.session->Stop();
    }
    stopped_ = true;
}
bool Robot::finished() const {
    if (stopped_ || (chassis_ && chassis_->finished()) ||
        (gimbal_ && gimbal_->finished()) || (ui_ && ui_->finished())) {
        return true;
    }
    for (const auto& [name, adapter] : adapters_) {
        if (adapter.started && !adapter.session->healthy()) {
            return true;
        }
    }
    return false;
}
std::string Robot::error() const {
    if (chassis_ && chassis_->finished() && !chassis_->error().empty()) {
        return "chassis: " + chassis_->error();
    }
    if (gimbal_ && gimbal_->finished() && !gimbal_->error().empty()) {
        return "gimbal: " + gimbal_->error();
    }
    if (ui_ && ui_->finished() && !ui_->error().empty()) {
        return "ui: " + ui_->error();
    }
    for (const auto& [name, adapter] : adapters_) {
        if (const auto error = adapter.session->error(); !error.empty()) {
            return "Adapter " + name + ": " + error;
        }
    }
    return {};
}
chassis::ChassisRuntime& Robot::chassis() {
    if (!chassis_) {
        throw std::logic_error("Chassis is disabled or not started");
    }
    return *chassis_;
}
gimbal::GimbalRuntime& Robot::gimbal() {
    if (!gimbal_) {
        throw std::logic_error("Gimbal is disabled or not started");
    }
    return *gimbal_;
}
ui::UiRuntime& Robot::ui() {
    if (!ui_) {
        throw std::logic_error("UI is disabled or not started");
    }
    return *ui_;
}
const wbr::host::UsbSession& Robot::adapter(const std::string& name) const {
    return *adapters_.at(name).session;
}
}  // namespace dust::app
