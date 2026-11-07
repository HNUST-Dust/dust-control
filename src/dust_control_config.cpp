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
    for (const auto* module : {&config.chassis, &config.gimbal, &config.ui}) {
        concurrency::ValidateThreadOptions(module->thread);
        if (!module->adapter.empty() &&
            !config.pdo.adapters.contains(module->adapter)) {
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
    if (yaml_validation::ParseUnsignedInteger(root["schema_version"], 4) != 4) {
        throw std::invalid_argument(
            "Unsupported application configuration version");
    }
    DustControlConfig config;
    config.motors_file = root["motors"].as<std::string>();
    const auto bus = root["ethercat"];
    yaml_validation::ValidateKeys(
        bus, {"master_index", "cycle_time_us", "limits", "thread"});
    config.ethercat_thread = ReadThread(bus["thread"]);
    config.pdo.master_index = static_cast<uint32_t>(
        yaml_validation::ParseUnsignedInteger(bus["master_index"], 255));
    config.pdo.cycle_time_us = static_cast<uint32_t>(
        yaml_validation::ParseUnsignedInteger(bus["cycle_time_us"], 1000));
    if (const auto limits = bus["limits"]) {
        yaml_validation::ValidateKeys(
            limits, {"max_rxpdo_bytes", "max_txpdo_bytes", "max_wire_time_us"});
        config.pdo.max_rxpdo_bytes =
            static_cast<uint32_t>(yaml_validation::ParseUnsignedInteger(
                limits["max_rxpdo_bytes"], 4096));
        config.pdo.max_txpdo_bytes =
            static_cast<uint32_t>(yaml_validation::ParseUnsignedInteger(
                limits["max_txpdo_bytes"], 4096));
        config.pdo.max_wire_time_us =
            static_cast<uint32_t>(yaml_validation::ParseUnsignedInteger(
                limits["max_wire_time_us"], 1000));
    }
    const auto execution = root["execution"];
    yaml_validation::ValidateKeys(execution,
                                  {"simulate", "realtime", "enable_output"});
    config.execution.simulate = execution["simulate"].as<bool>();
    config.execution.realtime = execution["realtime"].as<bool>();
    config.execution.enable_output = execution["enable_output"].as<bool>();
    const auto adapters = root["adapters"];
    if (!adapters.IsMap()) {
        throw std::invalid_argument("adapters must be a mapping");
    }
    for (const auto& entry : adapters) {
        const auto name = entry.first.as<std::string>();
        const auto node = entry.second;
        yaml_validation::ValidateKeys(node, {"alias", "position", "vendor_id",
                                             "product_code", "protocol"});
        if (node["protocol"].as<std::string>() != "dtg1") {
            throw std::invalid_argument("Only the DTG1 protocol is supported");
        }
        ethercat::AdapterPdoConfig pdo_adapter;
        pdo_adapter.alias = static_cast<uint16_t>(
            yaml_validation::ParseUnsignedInteger(node["alias"], 65535));
        pdo_adapter.position = static_cast<uint16_t>(
            yaml_validation::ParseUnsignedInteger(node["position"], 65535));
        pdo_adapter.vendor_id =
            static_cast<uint32_t>(yaml_validation::ParseUnsignedInteger(
                node["vendor_id"], 0xffffffffU));
        pdo_adapter.product_code =
            static_cast<uint32_t>(yaml_validation::ParseUnsignedInteger(
                node["product_code"], 0xffffffffU));
        if (!config.pdo.adapters.emplace(name, pdo_adapter).second) {
            throw std::invalid_argument("Duplicate PDO adapter name: " + name);
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
