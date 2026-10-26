/* SPDX-License-Identifier: Apache-2.0 */
#include "dust_control_config.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <set>
#include <stdexcept>

#include "yaml_validation.hpp"

namespace dust_control::app {
namespace {
concurrency::ThreadOptions ReadThread(const YAML::Node& node) {
    yaml_validation::ValidateKeys(node, {"cpu", "priority"});
    concurrency::ThreadOptions options;
    options.cpu = node["cpu"].IsNull() ? -1 : node["cpu"].as<int>();
    options.priority = node["priority"].as<int>();
    concurrency::ValidateThreadOptions(options);
    return options;
}
ModuleConfig ReadModule(const YAML::Node& node, bool chassis = false) {
    const std::string adapter_key = chassis ? "sensor_adapter" : "adapter";
    yaml_validation::ValidateKeys(node, {"enabled", adapter_key, "thread"});
    ModuleConfig config;
    config.enabled = node["enabled"].as<bool>();
    config.adapter =
        node[adapter_key].IsNull() ? "" : node[adapter_key].as<std::string>();
    config.thread = ReadThread(node["thread"]);
    return config;
}
}  // namespace
void ValidateDustControlConfig(const DustControlConfig& config) {
    if (config.execution.simulate && config.execution.enable_output) {
        throw std::invalid_argument("Simulation cannot enable real outputs");
    }
    if (config.execution.enable_output && !config.chassis.enabled) {
        throw std::invalid_argument(
            "Output requested without a chassis controller");
    }
    if (!config.chassis.enabled && !config.gimbal.enabled &&
        !config.ui.enabled) {
        throw std::invalid_argument("At least one module must be enabled");
    }
    (void)ethercat::BuildPdoLayout(config.pdo);
    concurrency::ValidateThreadOptions(config.ethercat_thread);
    if (config.adapters.size() != config.pdo.adapters.size()) {
        throw std::invalid_argument("Application/PDO adapter sets must match");
    }
    for (const auto& [name, adapter] : config.adapters) {
        if (!config.pdo.adapters.contains(name)) {
            throw std::invalid_argument("Missing PDO adapter: " + name);
        }
        adapter.devices.uart_sensors.Validate();
        const auto& capacity = config.pdo.adapters.at(name);
        for (const auto port : {adapter.devices.uart_sensors.remote,
                                adapter.devices.uart_sensors.hi91,
                                adapter.devices.uart_sensors.referee}) {
            if (port && !capacity.uart[port - 1].rx_bytes) {
                throw std::invalid_argument(
                    "Sensor binding requires UART RX capacity");
            }
        }
    }
    for (const auto* module : {&config.chassis, &config.gimbal, &config.ui}) {
        concurrency::ValidateThreadOptions(module->thread);
        if (!module->adapter.empty() &&
            !config.adapters.contains(module->adapter)) {
            throw std::invalid_argument("Unknown module adapter: " +
                                        module->adapter);
        }
    }
    if (config.chassis.enabled && config.chassis.adapter.empty()) {
        throw std::invalid_argument("Chassis requires an adapter binding");
    }
    if (config.chassis.enabled && config.motors_file.empty()) {
        throw std::invalid_argument("Chassis requires a motors file");
    }
}
DustControlConfig LoadDustControlConfig(const std::string& file) {
    const auto root = YAML::LoadFile(file);
    yaml_validation::ValidateKeys(
        root, {"schema_version", "execution", "adapters", "modules", "motors",
               "ethercat"});
    if (yaml_validation::ParseUnsignedInteger(root["schema_version"], 3) != 3) {
        throw std::invalid_argument(
            "Unsupported application configuration version");
    }
    DustControlConfig config;
    config.motors_file = root["motors"].as<std::string>();
    const auto bus = root["ethercat"];
    yaml_validation::ValidateKeys(bus, {"pdo_file", "thread"});
    config.ethercat_thread = ReadThread(bus["thread"]);
    config.pdo =
        ethercat::LoadPdoConfig((std::filesystem::path(file).parent_path() /
                                 bus["pdo_file"].as<std::string>())
                                    .string());
    const auto execution = root["execution"];
    yaml_validation::ValidateKeys(execution,
                                  {"simulate", "realtime", "enable_output"});
    config.execution.simulate = execution["simulate"].as<bool>();
    config.execution.realtime = execution["realtime"].as<bool>();
    config.execution.enable_output = execution["enable_output"].as<bool>();
    config.adapters.clear();
    const auto adapters = root["adapters"];
    if (!adapters.IsMap()) {
        throw std::invalid_argument("adapters must be a mapping");
    }
    for (const auto& entry : adapters) {
        const auto name = entry.first.as<std::string>();
        const auto node = entry.second;
        yaml_validation::ValidateKeys(node, {"uart_sensors"},
                                      {"strict_hi91_crc"});
        AdapterConfig adapter;
        if (node["strict_hi91_crc"]) {
            adapter.devices.strict_hi91_crc =
                node["strict_hi91_crc"].as<bool>();
        }
        if (const auto sensors = node["uart_sensors"]) {
            yaml_validation::ValidateKeys(sensors,
                                          {"remote", "hi91", "referee"});
            const auto channel = [](const YAML::Node& value) -> uint8_t {
                if (value.IsNull()) {
                    return 0;
                }
                const auto parsed =
                    yaml_validation::ParseUnsignedInteger(value, 6);
                if (!parsed) {
                    throw std::invalid_argument(
                        "UART channel must be 1..6 or null");
                }
                return static_cast<uint8_t>(parsed);
            };
            adapter.devices.uart_sensors = {channel(sensors["remote"]),
                                            channel(sensors["hi91"]),
                                            channel(sensors["referee"])};
        }
        if (!config.adapters.emplace(name, std::move(adapter)).second) {
            throw std::invalid_argument("Duplicate adapter name: " + name);
        }
    }
    const auto modules = root["modules"];
    yaml_validation::ValidateKeys(modules, {"chassis", "gimbal", "ui"});
    config.chassis = ReadModule(modules["chassis"], true);
    config.gimbal = ReadModule(modules["gimbal"]);
    config.ui = ReadModule(modules["ui"]);
    ValidateDustControlConfig(config);
    return config;
}
}  // namespace dust_control::app
