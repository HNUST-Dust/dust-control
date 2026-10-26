/* SPDX-License-Identifier: Apache-2.0 */
#include <array>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "dust_control.hpp"
#include "gimbal/gimbal_runtime.hpp"
#include "hardware/usb/usb_session.hpp"
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
        dust_control::gimbal::GimbalRuntime gimbal;
        dust_control::ui::UiRuntime ui;
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
        dust_control::gimbal::GimbalRuntime stopped;
        stopped.Stop();
        RejectRestart(stopped);
        // 析构必须唤醒等待中的线程，无需调用者预先 Stop。
        dust_control::ui::UiRuntime scope;
        scope.Start();
    }
    dust_control::host::UsbOptions usb_options;
    usb_options.realtime = false;
    dust_control::host::UsbSession board(usb_options, {}, {});
    {
        dust_control::ui::UiRuntime ui(
            {&board.link().referee_decoder(),
             dust_control::host::UartFramePort(board.transmitter())});
        dust_control::gimbal::GimbalRuntime gimbal(
            {&board.link().motor_decoder(), nullptr,
             dust_control::host::CanCommandPort(
                 board.transmitter(),
                 dust_control::host::CanProducer::kGimbal)});
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
        dust_control::host::UsbSession pending(usb_options, {}, {});
        const dust_control::host::CanCommandPort can(
            pending.transmitter(), dust_control::host::CanProducer::kGimbal);
        {
            dust_control::gimbal::GimbalRuntime gimbal(
                {&pending.link().motor_decoder(), nullptr, can});
            gimbal.Start();
        }
        // 空云台没有提前注册来源；真实算法就绪后仍可注册自己的命令组。
        const std::array groups{dust_control::host::CanCommandGroup{2, 0x200}};
        assert(can.Configure(groups, 20000000));
        bool rejected = false;
        try {
            dust_control::ui::UiRuntime invalid(
                {nullptr,
                 dust_control::host::UartFramePort(pending.transmitter())});
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        assert(rejected);
    }
    {
        dust_control::app::DustControlConfig config;
        config.execution.simulate = true;
        config.execution.realtime = false;
        dust_control::app::DustControl application(config, argv[1]);
        application.Start();
        WaitForWorkers();
        assert(!application.gimbal().attached() &&
               !application.ui().attached());
        application.Stop();
        assert(application.gimbal().finished() && application.ui().finished());
        WaitForWorkers(false);
        assert(application.error().empty());
    }
    {
        dust_control::app::DustControlConfig config;
        config.execution.realtime = false;
        dust_control::app::DustControl application(config, argv[1]);
        application.Start();
        WaitForWorkers();
        assert(application.ui().attached() && !application.gimbal().attached());
        application.Stop();
        assert(application.error().empty());
    }
}
