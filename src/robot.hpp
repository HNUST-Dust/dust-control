/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <map>
#include <memory>
#include <string>

#include "chassis/chassis_runtime.hpp"
#include "gimbal/gimbal_runtime.hpp"
#include "hardware/imu/imu_config.hpp"
#include "hardware/motors/motor_config.hpp"
#include "hardware/remote/remote_config.hpp"
#include "hardware/usb/usb_session.hpp"
#include "robot_config.hpp"
#include "ui/ui_runtime.hpp"

namespace dust_control::app {
// ROS 节点使用的装配处；只协调启动、故障和退出，没有业务循环或工作线程。
class Robot {
   public:
    explicit Robot(const std::string& config_directory);
    // 数值配置入口用于离线装配；设备文件相对配置目录解析。
    explicit Robot(RobotConfig config,
                   const std::string& config_directory = "");
    ~Robot();
    void Start();
    void Stop();
    bool finished() const;
    std::string error() const;
    // Start 成功后访问模块自己的接口。
    chassis::ChassisRuntime& chassis();
    gimbal::GimbalRuntime& gimbal();
    ui::UiRuntime& ui();
    const RobotConfig& configuration() const {
        return config_;
    }
    // 只读检查已装配的会话；绑定空模块的会话可能尚未启动。
    const dust_control::host::UsbSession& adapter(
        const std::string& name) const;

   private:
    void CreateAdapters();
    void CreateChassis();
    void CreateGimbal();
    void CreateUi();
    void StartAdapters();
    void StartModulesFor(const std::string& adapter);
    dust_control::host::UsbSession* SessionFor(const std::string& name);
    dust_control::modules::ChassisConfig chassis_config_;
    RobotConfig config_;
    devices::MotorConfig chassis_motors_;
    std::map<std::string, devices::MotorConfig> motor_configs_;
    devices::ImuConfig imu_;
    devices::RemoteConfig remote_;
    struct Adapter {
        std::unique_ptr<dust_control::host::UsbSession> session;
        std::atomic<bool> started{false};
        explicit Adapter(std::unique_ptr<dust_control::host::UsbSession> value)
            : session(std::move(value)) {}
    };
    std::map<std::string, Adapter> adapters_;
    std::atomic<bool> startup_ready_{false};
    std::unique_ptr<chassis::ChassisRuntime> chassis_;
    std::unique_ptr<gimbal::GimbalRuntime> gimbal_;
    std::unique_ptr<ui::UiRuntime> ui_;
    bool started_ = false;
    bool stopped_ = false;
};
}  // namespace dust_control::app
