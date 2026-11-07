/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "chassis_controller.h"
#include "chassis_observation.hpp"
#include "concurrency/thread.hpp"
#include "hardware/motors/motor_config.hpp"

namespace dust_control::devices {
class MotorFeedbackDecoder;
class WflySbusDecoder;
class Hi91Imu;
}  // namespace dust_control::devices
namespace dust_control::host {
class EthercatTransmitter;
class MotorCommandEncoder;
}  // namespace dust_control::host
namespace dust_control::chassis {
struct ChassisRuntimeOptions {
    concurrency::ThreadOptions thread{true, 3, 80};
    bool enable_output = false;
    // 借用整机启动许可；为空时由独立调用者直接管理输出策略。
    const std::atomic<bool>* startup_ready = nullptr;
};
// 借用本板设备、编码器和发送器；借用对象须长于底盘线程寿命。
// motors 仅包含本板电机，并与会话编码器使用的接线配置一致。
struct ChassisMotorAdapter {
    std::string name;
    const devices::MotorFeedbackDecoder* feedback = nullptr;
    dust_control::host::EthercatTransmitter* transmitter = nullptr;
    devices::MotorConfig motors;
    // DustControl 在握手和发送线程就绪后置 true；独立调用者可省略。
    const std::atomic<bool>* started = nullptr;
    const dust_control::host::MotorCommandEncoder* encoder = nullptr;
};
struct ChassisDevices {
    std::vector<ChassisMotorAdapter> motor_adapters;
    const devices::WflySbusDecoder* remote = nullptr;
    const devices::Hi91Imu* imu = nullptr;
};
class ChassisRuntime {
   public:
    ChassisRuntime(ChassisRuntimeOptions options,
                   const devices::MotorConfig& motors,
                   dust_control::chassis::InputDeadlines deadlines,
                   ChassisDevices devices = {},
                   dust_control::chassis::ChassisConfig config = {});
    ~ChassisRuntime();
    void Start();
    // 只停止并回收本模块线程，不停止借用的 EtherCAT 资源。
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
    // 更新绝对唤醒时间；错过周期不补发，超过看门狗期限时抛出异常。
    bool WaitForNextCycle(uint64_t& release, unsigned& late_cycles);
    ControllerInputs ReadInputs() const;
    // 先完成所有板的编码再逐板提交；提交失败时由工作线程故障退出。
    void SubmitMotorTargets(const ChassisControlOutput& output,
                            uint64_t command_sequence, uint64_t tick_start);
    void PublishObservation(ChassisObservation& sample, uint64_t tick_start,
                            uint64_t observation_sequence, unsigned late_cycles,
                            uint64_t& observation_drops);
    ChassisRuntimeOptions options_;
    ChassisDevices devices_;
    dust_control::chassis::ChassisController controller_;
    ChassisObservationQueue observations_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> finished_{false};
    std::thread worker_;
    std::string error_;
};
}  // namespace dust_control::chassis
