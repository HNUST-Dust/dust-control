/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <map>
#include <memory>
#include <string>

#include "chassis/chassis_runtime.hpp"
#include "dust_control_config.hpp"
#include "gimbal/gimbal_runtime.hpp"
#include "hardware/ethercat/master.hpp"
#include "hardware/imu/imu_config.hpp"
#include "hardware/motors/motor_config.hpp"
#include "hardware/remote/remote_config.hpp"
#include "ui/ui_runtime.hpp"

namespace dust_control::app {
// ROS 节点使用的装配处；只协调启动、故障和退出，没有业务循环或工作线程。
class DustControl {
   public:
    explicit DustControl(const std::string& config_directory);
    // 数值配置入口用于离线装配；设备文件相对配置目录解析。
    explicit DustControl(DustControlConfig config,
                         const std::string& config_directory = "");
    ~DustControl();
    void Start();
    void Stop();
    bool finished() const;
    std::string error() const;
    // Start 成功后访问模块自己的接口。
    chassis::ChassisRuntime& chassis();
    gimbal::GimbalRuntime& gimbal();
    ui::UiRuntime& ui();
    const DustControlConfig& configuration() const {
        return config_;
    }
    // 只读检查已装配的会话；绑定空模块的会话可能尚未启动。
    const dust_control::host::EthercatSession& adapter(
        const std::string& name) const;

   private:
    void CreateAdapters();
    void CreateChassis();
    void CreateGimbal();
    void CreateUi();
    void StartAdapters();
    void StartModulesFor(const std::string& adapter);
    dust_control::host::EthercatSession* SessionFor(const std::string& name);
    dust_control::chassis::ChassisConfig chassis_config_;
    DustControlConfig config_;
    devices::MotorConfig chassis_motors_;
    std::map<std::string, devices::MotorConfig> motor_configs_;
    devices::ImuConfig imu_;
    devices::RemoteConfig remote_;
    struct Adapter {
        std::unique_ptr<dust_control::host::EthercatSession> session;
        std::atomic<bool> started{false};
        explicit Adapter(
            std::unique_ptr<dust_control::host::EthercatSession> value)
            : session(std::move(value)) {}
    };
    std::map<std::string, Adapter> adapters_;
    std::unique_ptr<ethercat::EthercatMaster> master_;
    std::atomic<bool> startup_ready_{false};
    std::unique_ptr<chassis::ChassisRuntime> chassis_;
    std::unique_ptr<gimbal::GimbalRuntime> gimbal_;
    std::unique_ptr<ui::UiRuntime> ui_;
    bool started_ = false;
    bool stopped_ = false;
};
}  // namespace dust_control::app
