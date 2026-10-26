/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "concurrency/thread.hpp"
#include "hardware/ethercat/commands.hpp"

namespace dust_control::devices {
class MotorFeedbackDecoder;
class Hi91Decoder;
}  // namespace dust_control::devices
namespace dust_control::gimbal {
// 接口全部借用，必须长于本模块线程寿命；为空时离线运行。
// 空线程可先绑定 CAN 端口；motors 为空表示尚无电机映射，attached 不代表可控制。
struct GimbalDevices {
    const devices::MotorFeedbackDecoder* motors = nullptr;
    const devices::Hi91Decoder* imu = nullptr;
    std::optional<dust_control::host::CanCommandPort> can;
};
// 当前仅建立独立线程与生命周期；等待停止，不生成命令或维持虚假心跳。
class GimbalRuntime {
   public:
    explicit GimbalRuntime(GimbalDevices devices = {},
                           concurrency::ThreadOptions thread = {});
    ~GimbalRuntime();
    void Start();
    void Stop();
    bool finished() const {
        return finished_.load();
    }
    bool attached() const {
        return devices_.can.has_value();
    }
    // 仅在 finished() 或 Stop() 后读取。
    const std::string& error() const {
        return error_;
    }

   private:
    void Run();
    const GimbalDevices devices_;
    const concurrency::ThreadOptions thread_;
    std::mutex mutex_;
    std::condition_variable changed_;
    bool started_ = false;
    bool stop_ = false;
    std::atomic<bool> finished_{false};
    std::string error_;
    std::thread worker_;
};
}  // namespace dust_control::gimbal
