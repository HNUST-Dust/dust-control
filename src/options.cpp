/* SPDX-License-Identifier: Apache-2.0 */
#include "options.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace dust::app {
Options ParseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string option = argv[i];
        if (option == "--simulate") {
            options.simulate = true;
            continue;
        }
        if (option == "--foxglove") {
            options.foxglove_port = 8765;
            continue;
        }
        if (option == "--no-rt") {
            options.usb.realtime = false;
            continue;
        }
        if (option == "--enable-output") {
            options.enable_output = true;
            continue;
        }
        if (option == "--strict-hi91-crc") {
            options.usb.strict_hi91_crc = true;
            continue;
        }
        if (option == "--help") {
            std::cout << "chassis_controller [--cpu 3] [--usb-cpu 3] "
                         "[--priority 80] "
                         "[--usb-product NAME] [--usb-serial FULL_UID] "
                         "[--usb-pid PID] [--config DIR] [--seconds N] "
                         "[--foxglove] [--foxglove-port 8765]\n"
                      << "  Default: disabled actuator policy; remote "
                         "disabled pose must "
                         "be observed\n"
                      << "  after startup before --enable-output can honor "
                         "the remote "
                         "enable switch.\n"
                      << "  --simulate --no-rt runs the real core without "
                         "opening USB.\n";
            options.help = true;
            return options;
        }
        if (++i == argc) {
            throw std::runtime_error("Missing value for " + option);
        }
        if (option == "--config") {
            options.config_directory = argv[i];
        } else if (option == "--usb-product") {
            options.usb.product = argv[i];
        } else if (option == "--usb-serial") {
            options.usb.serial = argv[i];
        } else if (option == "--usb-pid") {
            const std::string value = argv[i];
            size_t end = 0;
            const auto pid = std::stoul(value, &end, 0);
            if (end != value.size() || pid > 0xffff || value.front() == '-') {
                throw std::runtime_error("Invalid USB PID");
            }
            options.usb.pid = static_cast<uint16_t>(pid);
        } else if (option == "--cpu") {
            options.control_cpu = std::stoi(argv[i]);
        } else if (option == "--usb-cpu") {
            options.usb.cpu = std::stoi(argv[i]);
        } else if (option == "--priority") {
            options.usb.priority = std::stoi(argv[i]);
        } else if (option == "--foxglove-port") {
            options.foxglove_port = std::stoi(argv[i]);
        } else if (option == "--telemetry-port") {
            options.telemetry_port = std::stoi(argv[i]);
        } else if (option == "--seconds") {
            options.seconds = std::stod(argv[i]);
        } else {
            throw std::runtime_error("Unknown option " + option);
        }
    }
    if (options.simulate && options.seconds == 0) {
        options.seconds = 1;
    }
    if (options.simulate && options.enable_output) {
        throw std::runtime_error("Simulation cannot enable real outputs");
    }
    ValidateOptions(options);
    return options;
}

void ValidateOptions(const Options& options) {
    if (options.control_cpu < 0 || options.control_cpu >= CPU_SETSIZE ||
        options.usb.cpu < 0 || options.usb.cpu >= CPU_SETSIZE ||
        options.usb.priority < 1 || options.usb.priority > 99 ||
        options.seconds < 0 || options.seconds > 86400 ||
        !std::isfinite(options.seconds) || options.telemetry_port < 0 ||
        options.telemetry_port > 65535 || options.foxglove_port < 0 ||
        options.foxglove_port > 65535) {
        throw std::runtime_error("Invalid runtime options");
    }
    if (options.command_timeout_ns == 0) {
        throw std::runtime_error("Command timeout must be positive");
    }
    if (options.simulate && options.enable_output) {
        throw std::runtime_error("Simulation cannot enable real outputs");
    }
}
}  // namespace dust::app
