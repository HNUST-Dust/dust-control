/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <string>

#include "concurrency/thread.hpp"
#include "hardware/ethercat/pdo_config.hpp"
namespace dust_control::app {
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
    ModuleConfig chassis{true, "base", {true, 3, 80}};
    ModuleConfig gimbal;
    ModuleConfig ui{true, "base", {}};
};
DustControlConfig LoadDustControlConfig(const std::string& file);
void ValidateDustControlConfig(const DustControlConfig& config);
}  // namespace dust_control::app
