/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <cstdint>
#include <hardware/usb/usb_link.hpp>
#include <string>

namespace dust::app {
struct Options {
    bool simulate = false;
    bool enable_output = false;
    bool help = false;
    double seconds = 0;
    std::string config_directory;
    int control_cpu = 3;
    int telemetry_port = 0;
    int foxglove_port = 0;
    bool ros_commands = false;
    uint64_t command_timeout_ns = 200000000;
    wbr::host::UsbOptions usb;
};
Options ParseOptions(int argc, char** argv);
void ValidateOptions(const Options& options);
}  // namespace dust::app
