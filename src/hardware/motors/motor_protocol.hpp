/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "concurrency/published_snapshot.hpp"
#include "hardware/can_frame.hpp"
#include "motor_config.hpp"

namespace dust_control::devices {
enum class ActuatorId : uint8_t {
    kLeftWheel,
    kRightWheel,
    kLeftJointB,
    kLeftJointD,
    kRightJointB,
    kRightJointD
};
enum class MotorMode { kNone, kTorque, kDisable, kClearError, kEnable };
struct MotorTarget {
    MotorMode mode = MotorMode::kNone;
    double torque_nm = 0.0;
};
// 每周期重新生成；每个执行器最多一项操作，kNone 表示本周期不发送。
// 轮力矩使用机体坐标；关节力矩保留现有电机坐标。
using MotorTargets = std::array<MotorTarget, 6>;

// 设备驱动产生的物理反馈；时间戳使用主机单调时钟。
struct MotorFeedback {
    uint64_t precise_timestamp_us = 0;
    bool valid = false;
    double position_rad = 0.0;
    double velocity_rad_s = 0.0;
    double torque_nm = 0.0;
    double speed_rpm = 0.0;
    bool enabled = false;
};

// 输入字节仅在调用期间借用，返回值拥有自己的物理量。
MotorFeedback DecodeMotorFeedback(unsigned index, const uint8_t* bytes,
                                  size_t size, uint64_t sample_us,
                                  const MotorConfig& config);

// Read 可跨线程调用；CAN 路由与解码由接收线程独占，不依赖 USB 格式。
class MotorFeedbackDecoder {
   public:
    explicit MotorFeedbackDecoder(MotorConfig config) : config_(config) {
        ValidateMotorAdapterConfig(config_);
    }
    void Reset() {
        latest_ = {};
        published_.Publish(latest_);
    }
    void Consume(uint8_t bus, uint16_t id, std::span<const uint8_t> bytes,
                 uint64_t sample_us, bool timestamp_valid);
    std::array<MotorFeedback, 6> Read() const {
        return published_.Read();
    }

   private:
    const MotorConfig config_;
    std::array<MotorFeedback, 6> latest_{};
    concurrency::PublishedSnapshot<std::array<MotorFeedback, 6>> published_;
};

// 单块 Adapter 的电机编码结果；批次容量属于电机编码接口，不限制通用 CAN 帧。
struct CanTransmitBatch {
    std::array<CanFrame, 6> frames{};
    size_t count = 0;
};

// 无状态编码；不持有 USB、回调或底盘控制器，不执行使能许可判断。
// 成功返回本周期 CAN 帧；无效目标返回 nullopt，不输出部分批次。
class MotorEncoder {
   public:
    explicit MotorEncoder(MotorConfig config) : config_(config) {
        ValidateMotorAdapterConfig(config_);
    }
    std::optional<CanTransmitBatch> Encode(const MotorTargets& targets) const;

   private:
    const MotorConfig config_;
};

}  // namespace dust_control::devices
