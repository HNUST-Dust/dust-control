/* SPDX-License-Identifier: Apache-2.0 */
#include "robot_config.hpp"

#include <algorithm>
#include <cctype>
#include <set>
#include <stdexcept>

#include "yaml_validation.hpp"

namespace dust::app {
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
void ValidateRobotConfig(const RobotConfig& config) {
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
    std::set<std::string> serials;
    for (const auto& [name, adapter] : config.adapters) {
        // Linux 线程名最多 15 字节，名称同时用于 usb-<name>-rx/tx。
        if (name.empty() || name.size() > 8 ||
            !std::all_of(name.begin(), name.end(), [](unsigned char value) {
                return std::isalnum(value) || value == '_';
            })) {
            throw std::invalid_argument(
                "Adapter name must contain 1..8 letters, digits or "
                "underscores");
        }
        if (config.adapters.size() > 1 && adapter.usb.serial.empty()) {
            throw std::invalid_argument(
                "Multiple adapters require explicit USB serials");
        }
        if (!adapter.usb.serial.empty() &&
            !serials.insert(adapter.usb.serial).second) {
            throw std::invalid_argument("Duplicate adapter USB serial");
        }
        adapter.usb.uart_sensors.Validate();
        concurrency::ValidateThreadOptions(
            {true, adapter.usb.cpu, adapter.usb.priority});
        concurrency::ValidateThreadOptions(
            {true, adapter.transmit.cpu, adapter.transmit.priority});
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
RobotConfig LoadRobotConfig(const std::string& file) {
    const auto root = YAML::LoadFile(file);
    yaml_validation::ValidateKeys(
        root, {"schema_version", "execution", "adapters", "modules", "motors"});
    if (yaml_validation::ParseUnsignedInteger(root["schema_version"], 2) != 2) {
        throw std::invalid_argument("Unsupported robot configuration version");
    }
    RobotConfig config;
    config.motors_file = root["motors"].as<std::string>();
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
        yaml_validation::ValidateKeys(
            node, {"serial", "receive_thread", "send_thread"},
            {"product", "pid", "strict_hi91_crc", "uart_sensors"});
        AdapterConfig adapter;
        adapter.usb.serial = node["serial"].as<std::string>();
        if (node["product"]) {
            adapter.usb.product = node["product"].as<std::string>();
        }
        if (node["pid"] && !node["pid"].IsNull()) {
            adapter.usb.pid = uint16_t(
                yaml_validation::ParseUnsignedInteger(node["pid"], 65535));
        }
        if (node["strict_hi91_crc"]) {
            adapter.usb.strict_hi91_crc = node["strict_hi91_crc"].as<bool>();
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
            adapter.usb.uart_sensors = {channel(sensors["remote"]),
                                        channel(sensors["hi91"]),
                                        channel(sensors["referee"])};
        }
        const auto receive = ReadThread(node["receive_thread"]);
        const auto send = ReadThread(node["send_thread"]);
        adapter.usb.cpu = receive.cpu;
        adapter.usb.priority = receive.priority;
        adapter.transmit.cpu = send.cpu;
        adapter.transmit.priority = send.priority;
        if (!config.adapters.emplace(name, std::move(adapter)).second) {
            throw std::invalid_argument("Duplicate adapter name: " + name);
        }
    }
    const auto modules = root["modules"];
    yaml_validation::ValidateKeys(modules, {"chassis", "gimbal", "ui"});
    config.chassis = ReadModule(modules["chassis"], true);
    config.gimbal = ReadModule(modules["gimbal"]);
    config.ui = ReadModule(modules["ui"]);
    ValidateRobotConfig(config);
    return config;
}
}  // namespace dust::app
