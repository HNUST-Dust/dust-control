/* SPDX-License-Identifier: Apache-2.0 */
#include <chrono>
#include <csignal>
#include <iostream>
#include <options.hpp>
#include <thread>

#include "robot.hpp"

namespace {
volatile std::sig_atomic_t stopping = 0;
void Signal(int) {
    stopping = 1;
}
}  // namespace
int main(int argc, char** argv) {
    try {
        const auto options = dust::app::ParseOptions(argc, argv);
        if (options.help) {
            return 0;
        }
        std::signal(SIGINT, Signal);
        std::signal(SIGTERM, Signal);
        dust::app::Robot robot(options);
        robot.Start();
        while (!stopping && !robot.finished()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        robot.Stop();
        if (!robot.error().empty()) {
            throw std::runtime_error(robot.error());
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
