/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "commands.hpp"
#include "device_sensor_page.hpp"
#include "hardware/imu/hi91_imu.hpp"
#include "hardware/imu/onboard_imu.hpp"
#include "hardware/remote/wfly_sbus_protocol.h"

namespace dust_control::host {
class EthercatSession {
   public:
    explicit EthercatSession(const ethercat::AdapterPdoLayout& layout,
                             const devices::MotorConfig& motors = {});
    bool healthy() const {
        return transmitter_.healthy();
    }
    std::string error() const {
        return transmitter_.error();
    }
    EthercatTransmitter& transmitter() {
        return transmitter_;
    }
    const EthercatTransmitter& transmitter() const {
        return transmitter_;
    }
    const MotorCommandEncoder& motor_encoder() const {
        return motor_encoder_;
    }
    const devices::MotorFeedbackDecoder& motor_decoder() const {
        return motors_;
    }
    const devices::WflySbusDecoder& wfly_sbus_decoder() const {
        return remote_;
    }
    const devices::Hi91Imu& hi91_imu() const {
        return imu_;
    }
    const devices::OnboardImu& onboard_imu() const {
        return onboard_imu_;
    }
    // 以下状态和周期入口只由主站线程访问，不执行总线 I/O。
    bool discovered() const {
        return discovered_;
    }
    bool active() const {
        return active_;
    }
    bool Receive(std::span<const uint8_t> image, uint64_t now_ns,
                 uint64_t session_id);
    bool Encode(std::span<uint8_t> image, uint64_t now_ns, uint64_t session_id,
                uint32_t flags);
    void Activate(uint64_t now_ns) {
        transmitter_.Activate(now_ns);
    }
    void Fail(const std::string& reason) {
        transmitter_.Fail(reason);
    }

   private:
    bool FailInput(const char* reason);
    EthercatTransmitter transmitter_;
    devices::MotorFeedbackDecoder motors_;
    MotorCommandEncoder motor_encoder_;
    devices::WflySbusDecoder remote_;
    devices::Hi91Imu imu_;
    devices::OnboardImu onboard_imu_;
    const bool decode_motors_;
    bool discovered_ = false, active_ = false;
    struct State {
        std::array<uint8_t, ethercat::pdo::kRxBytes> held{};
        std::array<uint8_t, 6> motor_sequence{};
        std::array<bool, 6> have_motor{};
        uint32_t command_source_sequence = 0, hi91_sequence = 0;
        uint32_t sbus_ack = 0, imu_sequence = 0;
        uint8_t sequence = 0, ack_repeats = 0;
        bool have_command = false, have_hi91 = false;
        bool have_sbus = false, have_imu = false;
    } state_;
};
}  // namespace dust_control::host
