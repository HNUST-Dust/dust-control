/* SPDX-License-Identifier: Apache-2.0 */
#include <array>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "gimbal/gimbal_runtime.hpp"
#include "hardware/usb/usb_session.hpp"
#include "robot.hpp"
#include "ui/ui_runtime.hpp"

namespace {
bool HasThread(const std::string& name) {
    for (const auto& entry :
         std::filesystem::directory_iterator("/proc/self/task")) {
        std::ifstream input(entry.path() / "comm");
        std::string actual;
        std::getline(input, actual);
        if (actual == name) {
            return true;
        }
    }
    return false;
}
void WaitForWorkers(bool present = true) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    const auto ready = [present] {
        const bool gimbal = HasThread("dust-gimbal"), ui = HasThread("dust-ui");
        return present ? gimbal && ui : !gimbal && !ui;
    };
    while (!ready() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    assert(ready());
}
template <class Worker>
void RejectRestart(Worker& worker) {
    bool rejected = false;
    try {
        worker.Start();
    } catch (const std::logic_error&) {
        rejected = true;
    }
    assert(rejected);
}
}  // namespace
int main(int argc, char** argv) {
    assert(argc == 2);
    {
        dust::gimbal::GimbalRuntime gimbal;
        dust::ui::UiRuntime ui;
        gimbal.Start();
        ui.Start();
        WaitForWorkers();
        assert(!gimbal.attached() && !ui.attached());
        RejectRestart(gimbal);
        RejectRestart(ui);
        gimbal.Stop();
        ui.Stop();
        ui.Stop();
        assert(gimbal.finished() && ui.finished());
        assert(gimbal.error().empty() && ui.error().empty());
        RejectRestart(gimbal);
        RejectRestart(ui);
        WaitForWorkers(false);
    }
    {
        dust::gimbal::GimbalRuntime stopped;
        stopped.Stop();
        RejectRestart(stopped);
        // 析构必须唤醒等待中的线程，无需调用者预先 Stop。
        dust::ui::UiRuntime scope;
        scope.Start();
    }
    wbr::host::UsbOptions usb_options;
    usb_options.realtime = false;
    wbr::host::UsbSession board(usb_options, {}, {});
    {
        dust::ui::UiRuntime ui({&board.link().referee_decoder(),
                                wbr::host::UartFramePort(board.transmitter())});
        dust::gimbal::GimbalRuntime gimbal(
            {&board.link().motor_decoder(), nullptr,
             wbr::host::CanCommandPort(board.transmitter(),
                                       wbr::host::CanProducer::kGimbal)});
        ui.Start();
        gimbal.Start();
        WaitForWorkers();
        assert(ui.attached() && gimbal.attached());
    }
    // 空模块的 Stop/析构不能停止借用会话，且不能注册 CAN 来源维持心跳。
    assert(board.Start());
    assert(board.healthy());
    board.Stop();
    {
        wbr::host::UsbSession pending(usb_options, {}, {});
        const wbr::host::CanCommandPort can(pending.transmitter(),
                                            wbr::host::CanProducer::kGimbal);
        {
            dust::gimbal::GimbalRuntime gimbal(
                {&pending.link().motor_decoder(), nullptr, can});
            gimbal.Start();
        }
        // 空云台没有提前注册来源；真实算法就绪后仍可注册自己的命令组。
        const std::array groups{wbr::host::CanCommandGroup{2, 0x200}};
        assert(can.Configure(groups, 20000000));
        bool rejected = false;
        try {
            dust::ui::UiRuntime invalid(
                {nullptr, wbr::host::UartFramePort(pending.transmitter())});
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        assert(rejected);
    }
    {
        dust::app::RobotConfig config;
        config.execution.simulate = true;
        config.execution.realtime = false;
        dust::app::Robot robot(config, argv[1]);
        robot.Start();
        WaitForWorkers();
        assert(!robot.gimbal().attached() && !robot.ui().attached());
        robot.Stop();
        assert(robot.gimbal().finished() && robot.ui().finished());
        WaitForWorkers(false);
        assert(robot.error().empty());
    }
    {
        dust::app::RobotConfig config;
        config.execution.realtime = false;
        dust::app::Robot robot(config, argv[1]);
        robot.Start();
        WaitForWorkers();
        assert(robot.ui().attached() && !robot.gimbal().attached());
        robot.Stop();
        assert(robot.error().empty());
    }
}
