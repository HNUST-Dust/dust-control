/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "chassis_controller.h"
#include "chassis_observation.hpp"
#include "concurrency/thread.hpp"
#include "hardware/motors/motor_protocol.hpp"

namespace dust_control::devices {
class MotorFeedbackDecoder;
class WflySbusDecoder;
class Hi91Decoder;
}  // namespace dust_control::devices
namespace dust_control::host {
class UsbTransmitter;
}
namespace dust_control::chassis {
struct ChassisRuntimeOptions {
    concurrency::ThreadOptions thread{true, 3, 80};
    bool enable_output = false;
    // 借用整机启动许可；为空时由独立调用者直接管理输出策略。
    const std::atomic<bool>* startup_ready = nullptr;
};
// 借用本板设备和发送器，motors 仅包含本板电机；借用对象须长于底盘线程寿命。
struct ChassisMotorAdapter {
    std::string name;
    const devices::MotorFeedbackDecoder* feedback = nullptr;
    dust_control::host::UsbTransmitter* transmitter = nullptr;
    devices::MotorConfig motors;
    // Robot 在握手和发送线程就绪后置 true；独立调用者可省略。
    const std::atomic<bool>* started = nullptr;
};
struct ChassisDevices {
    std::vector<ChassisMotorAdapter> motor_adapters;
    const devices::WflySbusDecoder* remote = nullptr;
    const devices::Hi91Decoder* imu = nullptr;
};
class ChassisRuntime {
   public:
    ChassisRuntime(ChassisRuntimeOptions options,
                   const devices::MotorConfig& motors,
                   dust_control::modules::InputDeadlines deadlines,
                   ChassisDevices devices = {},
                   dust_control::modules::ChassisConfig config = {});
    ~ChassisRuntime();
    void Start();
    // 只停止并回收本模块线程，不停止借用的 USB 资源。
    void Stop();
    bool finished() const {
        return finished_.load();
    }
    // 仅在 finished() 或 Stop() 后读取。
    const std::string& error() const {
        return error_;
    }
    // 单个非实时消费者复制一条观测；队列为空时立即返回 false。
    bool TakeObservation(ChassisObservation& sample) {
        return observations_.Pop(sample);
    }

   private:
    void Run();
    const dust_control::modules::ChassisConfig config_;
    ChassisRuntimeOptions options_;
    ChassisDevices devices_;
    std::vector<devices::MotorEncoder> encoders_;
    dust_control::modules::ChassisController controller_;
    ChassisObservationQueue observations_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> finished_{false};
    std::thread worker_;
    std::string error_;
};
}  // namespace dust_control::chassis
