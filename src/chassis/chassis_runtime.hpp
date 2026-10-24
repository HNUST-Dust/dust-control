/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <atomic>
#include <string>
#include <thread>

#include "chassis_controller.h"
#include "hardware/motors/motor_encoder.hpp"
#include "motion_command.hpp"
#include "telemetry/telemetry.hpp"

namespace dust::devices {
class MotorFeedbackDecoder;
class SbusDecoder;
class Hi91Decoder;
}  // namespace dust::devices
namespace wbr::host {
class UsbTransmitter;
}
namespace dust::chassis {
struct ChassisRuntimeOptions {
    bool realtime = true;
    int cpu = 3;
    int priority = 80;
    bool enable_output = false;
    double seconds = 0;
    MotionCommandOptions commands;
};
// 全部为空表示离线；硬件运行要求四个借用接口同时存在并长于底盘线程寿命。
struct ChassisDevices {
    const devices::MotorFeedbackDecoder* motors = nullptr;
    const devices::SbusDecoder* remote = nullptr;
    const devices::Hi91Decoder* imu = nullptr;
    wbr::host::UsbTransmitter* transmitter = nullptr;
};
class ChassisRuntime {
   public:
    ChassisRuntime(ChassisRuntimeOptions options,
                   const devices::MotorConfig& motors,
                   modules::InputDeadlines deadlines,
                   ChassisDevices devices = {},
                   wbr::host::TelemetryPublisher* telemetry = nullptr,
                   modules::ChassisConfig config = {});
    ~ChassisRuntime();
    void Start();
    // 只停止并回收本模块线程，不停止借用的 USB 或遥测资源。
    void Stop();
    bool finished() const {
        return finished_.load();
    }
    // 仅在 finished() 或 Stop() 后读取。
    const std::string& error() const {
        return error_;
    }
    MotionCommandQueue& commands() {
        return commands_;
    }
    bool TakeObservation(wbr::host::TelemetrySample& sample) {
        return observations_.Pop(sample);
    }

   private:
    void Run();
    const modules::ChassisConfig config_;
    ChassisRuntimeOptions options_;
    ChassisDevices devices_;
    wbr::host::TelemetryPublisher* telemetry_;
    devices::MotorEncoder encoder_;
    modules::ChassisController controller_;
    MotionCommandQueue commands_;
    wbr::host::TelemetryQueue observations_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> finished_{false};
    std::thread worker_;
    std::string error_;
};
}  // namespace dust::chassis
