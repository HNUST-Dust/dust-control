/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <memory>
#include <string>

#include "chassis/chassis_runtime.hpp"
#include "hardware/imu/imu_config.hpp"
#include "hardware/motors/motor_config.hpp"
#include "hardware/remote/remote_config.hpp"
#include "hardware/usb/usb_session.hpp"
#include "options.hpp"

namespace dust::app {
// ROS 与离线入口共用的装配处；只协调启动、故障和退出，没有业务循环或工作线程。
class Robot {
   public:
    explicit Robot(const Options& options);
    ~Robot();
    void Start();
    void Stop();
    bool finished() const;
    std::string error() const;
    // Start 成功后访问模块自己的指令与观测接口。
    chassis::ChassisRuntime& chassis();

   private:
    Options options_;
    devices::MotorConfig motors_;
    devices::ImuConfig imu_;
    devices::RemoteConfig remote_;
    std::unique_ptr<wbr::host::UsbSession> usb_;
    std::unique_ptr<wbr::host::TelemetryPublisher> telemetry_;
    std::unique_ptr<chassis::ChassisRuntime> chassis_;
    bool started_ = false;
    bool stopped_ = false;
};
}  // namespace dust::app
