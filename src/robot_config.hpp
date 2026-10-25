/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <map>
#include <string>

#include "concurrency/thread.hpp"
#include "hardware/usb/usb_link.hpp"
#include "hardware/usb/usb_transmitter.hpp"

namespace dust::app {
struct AdapterConfig {
    wbr::host::UsbOptions usb;
    wbr::host::UsbTransmitOptions transmit;
};
struct ModuleConfig {
    bool enabled = true;
    std::string adapter;
    concurrency::ThreadOptions thread;
};
struct ExecutionConfig {
    bool simulate = false;
    bool realtime = true;
    bool enable_output = false;
};
// 描述启动方式与装配；控制参数、设备协议和线程循环仍归各模块所有。
struct RobotConfig {
    ExecutionConfig execution;
    std::string motors_file = "motors_params.yaml";
    std::map<std::string, AdapterConfig> adapters{{"base", {}}};
    ModuleConfig chassis{true, "base", {true, 3, 80}};
    ModuleConfig gimbal;
    ModuleConfig ui{true, "base", {}};
};
RobotConfig LoadRobotConfig(const std::string& file);
void ValidateRobotConfig(const RobotConfig& config);
}  // namespace dust::app
