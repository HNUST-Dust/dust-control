/* SPDX-License-Identifier: Apache-2.0 */
#include "dust_control.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace dust_control::app {
DustControl::DustControl(const std::string& config_directory)
    : DustControl(
          LoadDustControlConfig(
              (std::filesystem::path(config_directory) / "dust_control.yaml")
                  .string()),
          config_directory) {}
DustControl::DustControl(DustControlConfig config,
                         const std::string& config_directory)
    : config_(std::move(config)) {
    ValidateDustControlConfig(config_);
    const std::filesystem::path directory(config_directory);
    if (!directory.empty()) {
        if (config_.chassis.enabled) {
            chassis_config_ = dust_control::chassis::LoadChassisConfig(
                (directory / "chassis_params.yaml").string());
            remote_ = devices::LoadRemoteConfig(
                (directory / "sensors_params.yaml").string());
        }
        imu_ = devices::LoadImuConfig(
            (directory / "sensors_params.yaml").string());
    }
    // 配置先完整加载；此处不申请主站或启动线程。
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
DustControl::~DustControl() {
    Stop();
}
void DustControl::Start() {
    if (started_ || stopped_) {
        throw std::logic_error("DustControl cannot be restarted");
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
void DustControl::CreateAdapters() {
    if (config_.execution.simulate) {
        return;
    }
    auto thread = config_.ethercat_thread;
    thread.realtime = config_.execution.realtime;
    master_ = std::make_unique<ethercat::EthercatMaster>(config_.pdo, thread);
    const auto layout = ethercat::BuildPdoLayout(config_.pdo);
    for (const auto& [name, adapter] : config_.adapters) {
        auto devices = adapter.devices;
        const auto motors = motor_configs_.find(name);
        devices.decode_motors = motors != motor_configs_.end();
        auto session = std::make_unique<host::EthercatSession>(
            config_.pdo.adapters.at(name), layout.adapters.at(name), devices,
            devices.decode_motors ? motors->second : devices::MotorConfig{},
            imu_);
        master_->Attach(name, *session);
        adapters_.try_emplace(name, std::move(session));
    }
}

void DustControl::CreateChassis() {
    if (config_.chassis.enabled) {
        const auto& motors = chassis_motors_;
        chassis::ChassisRuntimeOptions control;
        control.thread = config_.chassis.thread;
        control.thread.realtime = config_.execution.realtime;
        control.enable_output = config_.execution.enable_output;
        control.startup_ready = &startup_ready_;
        chassis::ChassisDevices devices;
        if (auto* bus = SessionFor(config_.chassis.adapter)) {
            devices.remote = &bus->wfly_sbus_decoder();
            devices.imu = &bus->hi91_decoder();
        }
        for (const auto& [name, config] : motor_configs_) {
            if (auto* bus = SessionFor(name)) {
                if (std::any_of(config.present.begin(), config.present.end(),
                                [](bool value) { return value; })) {
                    devices.motor_adapters.push_back(
                        {name, &bus->motor_decoder(), &bus->transmitter(),
                         config, &adapters_.at(name).started});
                }
            }
        }
        chassis_ = std::make_unique<chassis::ChassisRuntime>(
            control, motors,
            dust_control::chassis::InputDeadlines{
                motors.freshness_us, imu_.freshness_us, remote_.freshness_us},
            devices, chassis_config_);
    }
}

void DustControl::CreateGimbal() {
    if (config_.gimbal.enabled) {
        gimbal::GimbalDevices devices;
        if (auto* bus = SessionFor(config_.gimbal.adapter)) {
            if (motor_configs_.contains(config_.gimbal.adapter)) {
                devices.motors = &bus->motor_decoder();
            }
            devices.imu = &bus->hi91_decoder();
            devices.can.emplace(bus->transmitter(),
                                dust_control::host::CanProducer::kGimbal);
        }
        auto thread = config_.gimbal.thread;
        thread.realtime = config_.execution.realtime;
        gimbal_ = std::make_unique<gimbal::GimbalRuntime>(devices, thread);
    }
}

void DustControl::CreateUi() {
    if (config_.ui.enabled) {
        ui::UiDevices devices;
        if (auto* bus = SessionFor(config_.ui.adapter)) {
            devices.referee = &bus->referee_decoder();
            devices.uart.emplace(bus->transmitter());
        }
        auto thread = config_.ui.thread;
        thread.realtime = config_.execution.realtime;
        ui_ = std::make_unique<ui::UiRuntime>(devices, thread);
    }
}

void DustControl::StartAdapters() {
    if (master_ && !master_->Start()) {
        throw std::runtime_error("EtherCAT startup: " + master_->error());
    }
    for (auto& [name, adapter] : adapters_) {
        adapter.started = true;
        std::cout << "EtherCAT adapter " << name << " ready" << std::endl;
    }
    for (const auto& [name, config] : config_.adapters) {
        (void)config;
        StartModulesFor(name);
    }
}

void DustControl::StartModulesFor(const std::string& adapter) {
    if (gimbal_ && config_.gimbal.adapter == adapter) {
        gimbal_->Start();
    }
    if (ui_ && config_.ui.adapter == adapter) {
        ui_->Start();
    }
}

dust_control::host::EthercatSession* DustControl::SessionFor(
    const std::string& name) {
    const auto found = adapters_.find(name);
    return found == adapters_.end() ? nullptr : found->second.session.get();
}

void DustControl::Stop() {
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
    if (master_) {
        master_->Stop();
    }
    stopped_ = true;
}
bool DustControl::finished() const {
    if (stopped_ || (chassis_ && chassis_->finished()) ||
        (gimbal_ && gimbal_->finished()) || (ui_ && ui_->finished())) {
        return true;
    }
    if (master_ && !master_->healthy()) {
        return true;
    }
    for (const auto& [name, adapter] : adapters_) {
        if (adapter.started && !adapter.session->healthy()) {
            return true;
        }
    }
    return false;
}
std::string DustControl::error() const {
    if (master_ && !master_->error().empty()) {
        return "EtherCAT: " + master_->error();
    }
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
chassis::ChassisRuntime& DustControl::chassis() {
    if (!chassis_) {
        throw std::logic_error("Chassis is disabled or not started");
    }
    return *chassis_;
}
gimbal::GimbalRuntime& DustControl::gimbal() {
    if (!gimbal_) {
        throw std::logic_error("Gimbal is disabled or not started");
    }
    return *gimbal_;
}
ui::UiRuntime& DustControl::ui() {
    if (!ui_) {
        throw std::logic_error("UI is disabled or not started");
    }
    return *ui_;
}
const dust_control::host::EthercatSession& DustControl::adapter(
    const std::string& name) const {
    return *adapters_.at(name).session;
}
}  // namespace dust_control::app
