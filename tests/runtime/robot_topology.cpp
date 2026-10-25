/* SPDX-License-Identifier: Apache-2.0 */
#include <sched.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <thread>

#include "robot.hpp"

namespace {
int FindThread(const std::string& name) {
    for (const auto& entry :
         std::filesystem::directory_iterator("/proc/self/task")) {
        std::ifstream input(entry.path() / "comm");
        std::string actual;
        std::getline(input, actual);
        if (actual == name) {
            return std::stoi(entry.path().filename());
        }
    }
    return -1;
}
void CheckThread(const std::string& name, int cpu) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    bool ready = false;
    do {
        const int tid = FindThread(name);
        if (tid > 0 && sched_getscheduler(tid) == SCHED_OTHER) {
            cpu_set_t actual;
            ready = cpu < 0 ||
                    (sched_getaffinity(tid, sizeof(actual), &actual) == 0 &&
                     CPU_COUNT(&actual) == 1 && CPU_ISSET(cpu, &actual));
        }
        if (!ready) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    } while (!ready && std::chrono::steady_clock::now() < deadline);
    assert(ready);
}
}  // namespace
int TestSplit(const std::string& directory) {
    dust_control::app::Robot robot{directory};
    try {
        robot.Start();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    const auto& config = robot.configuration();
    const auto all_motors = dust_control::devices::LoadMotorConfig(
        (std::filesystem::path(directory) / config.motors_file).string());
    const std::set<std::string> motor_adapters(all_motors.adapters.begin(),
                                               all_motors.adapters.end());
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    bool complete = false;
    dust_control::chassis::ChassisObservation sample;
    std::array<bool, 6> observed{};
    while (!robot.finished() && std::chrono::steady_clock::now() < deadline) {
        while (robot.chassis().TakeObservation(sample)) {
            for (size_t i = 0; i < observed.size(); ++i) {
                observed[i] =
                    observed[i] ||
                    sample.input.motor_feedback_age_us[i] != UINT32_MAX;
            }
        }
        complete = std::all_of(observed.begin(), observed.end(),
                               [](bool value) { return value; });
        if (complete)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (std::getenv("WBR_MOCK_DISCONNECT_INDEX")) {
        while (!robot.finished() &&
               std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        assert(robot.finished() && !robot.error().empty());
        std::cout << "Split adapter fault: " << robot.error() << '\n';
        robot.Stop();
        return 0;
    }
    const auto steady_end =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(80);
    while (!robot.finished() && std::chrono::steady_clock::now() < steady_end) {
        while (robot.chassis().TakeObservation(sample)) {
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (robot.finished()) {
        std::cerr << robot.error() << '\n';
    }
    assert(complete && !robot.finished());
    for (const auto& name : motor_adapters) {
        const auto expected =
            dust_control::devices::MotorConfigForAdapter(all_motors, name);
        const auto& usb = robot.adapter(name);
        assert(usb.healthy() && usb.transmitter().HasCanProducers());
        const auto feedback = usb.link().motor_decoder().Read();
        for (size_t i = 0; i < expected.present.size(); ++i) {
            assert(feedback[i].valid == expected.present[i]);
        }
    }
    if (motor_adapters.size() == 3) {
        const auto left = robot.adapter("head").link().motor_decoder().Read();
        const auto right = robot.adapter("aux").link().motor_decoder().Read();
        assert(left[2].position_rad != right[4].position_rad);
    }
    robot.Stop();
    if (!robot.error().empty()) {
        std::cerr << "After stop: " << robot.error() << '\n';
    }
    assert(robot.error().empty());
    for (const auto& name : motor_adapters)
        assert(!robot.adapter(name).healthy());
    std::cout << "Split motor routing and feedback passed\n";
    return 0;
}

int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "--split")
        return TestSplit(argv[2]);
    if (argc == 3 && std::string(argv[1]) == "--validate") {
        try {
            dust_control::app::Robot robot{std::string(argv[2])};
            return 0;
        } catch (const std::exception& error) {
            std::cerr << error.what() << '\n';
            return 1;
        }
    }
    assert(argc == 2);
    dust_control::app::Robot robot{std::string(argv[1])};
    robot.Start();
    const auto& config = robot.configuration();
    assert(config.adapters.size() == 2);
    assert(config.chassis.adapter == "drive" &&
           config.gimbal.adapter == "head");
    assert(robot.ui().attached() && robot.gimbal().attached());
    assert(robot.adapter("drive").healthy());
    assert(!robot.adapter("head").healthy());
    assert(robot.adapter("drive").transmitter().HasCanProducers());
    assert(!robot.adapter("head").transmitter().HasCanProducers());
    assert(&robot.adapter("drive").link().hi91_decoder() !=
           &robot.adapter("head").link().hi91_decoder());
    CheckThread("dust-chassis", config.chassis.thread.cpu);
    CheckThread("dust-gimbal", config.gimbal.thread.cpu);
    CheckThread("dust-ui", config.ui.thread.cpu);
    CheckThread("usb-drive-rx", config.adapters.at("drive").usb.cpu);
    CheckThread("usb-drive-tx", config.adapters.at("drive").transmit.cpu);
    assert(FindThread("usb-head-rx") < 0 && FindThread("usb-head-tx") < 0);
    robot.Stop();
    assert(robot.finished() && robot.error().empty());
    std::cout << "Two named adapters, explicit module binding, thread "
                 "placement, independent state and idle-board policy passed\n";
}
