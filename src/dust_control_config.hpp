/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <map>
#include <string>

#include "concurrency/thread.hpp"
#include "hardware/ethercat/session.hpp"
namespace dust_control::app {
struct AdapterConfig {
    host::EthercatDeviceOptions devices;
};
struct ModuleConfig {
    bool enabled = true;
    std::string adapter;
    concurrency::ThreadOptions thread;
};
struct ExecutionConfig {
    bool simulate = false, realtime = true, enable_output = false;
};
struct DustControlConfig {
    ExecutionConfig execution;
    std::string motors_file = "motors_params.yaml";
    ethercat::PdoConfig pdo;
    concurrency::ThreadOptions ethercat_thread{true, 3, 80};
    std::map<std::string, AdapterConfig> adapters{{"base", {}}};
    ModuleConfig chassis{true, "base", {true, 3, 80}};
    ModuleConfig gimbal;
    ModuleConfig ui{true, "base", {}};
};
DustControlConfig LoadDustControlConfig(const std::string& file);
void ValidateDustControlConfig(const DustControlConfig& config);
}  // namespace dust_control::app
