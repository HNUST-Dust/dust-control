/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstdint>
#include <string>

#include "dji_protocol.h"
#include "dm_protocol.h"

namespace dust::devices {
enum class MotorProtocol { kDji, kDm };
// 接线配置：由 YAML 或显式调用方提供，不内置机器人地址。
struct MotorAddress {
    uint8_t bus = 0;
    uint16_t command_id = 0;
    uint16_t feedback_id = 0;
    uint8_t command_slot = 0;
};
struct MotorConfig {
    std::array<std::string, 6> adapters{};
    std::array<bool, 6> present{};
    std::array<MotorProtocol, 6> protocols{
        MotorProtocol::kDji, MotorProtocol::kDji, MotorProtocol::kDm,
        MotorProtocol::kDm,  MotorProtocol::kDm,  MotorProtocol::kDm};
    std::array<MotorAddress, 6> addresses{};
    std::array<uint64_t, 6> freshness_us{3000, 3000, 3000, 3000, 3000, 3000};
    std::array<double, 2> wheel_command_sign{-1.0, 1.0};
    static constexpr double dji_current_per_nm =
        platform::motor_model::kDjiCurrentPerNm;
    static constexpr int16_t dji_protocol_current_limit =
        platform::motor_model::kDjiProtocolCurrentLimit;
    static constexpr protocols::DmMitRange dm_range =
        platform::motor_model::kJointMitRange;
    double wheel_torque_limit_nm() const {
        return static_cast<double>(dji_protocol_current_limit) /
               dji_current_per_nm;
    }
};
void ValidateMotorConfig(const MotorConfig& config);
// 单个 USB 会话的编解码器只能持有一块板的设备。
void ValidateMotorAdapterConfig(const MotorConfig& config);
// 启动时返回完整校验的配置值，不修改任何已存在的设备实例。
MotorConfig LoadMotorConfig(const std::string& file);
// 启动装配时从全量配置提取本板设备，运行期不再解析归属。
MotorConfig MotorConfigForAdapter(const MotorConfig& config,
                                  const std::string& adapter);
}  // namespace dust::devices
