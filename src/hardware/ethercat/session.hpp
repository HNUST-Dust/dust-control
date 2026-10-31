/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <atomic>
#include <optional>

#include "commands.hpp"
#include "hardware/imu/hi91_protocol.h"
#include "hardware/imu/onboard_imu.hpp"
#include "hardware/motors/motor_protocol.hpp"
#include "hardware/referee/referee_protocol.h"
#include "hardware/remote/wfly_sbus_protocol.h"
#include "uart_receive_queue.hpp"

namespace dust_control::host {
struct UartSensorBindings {
    uint8_t remote = 1, hi91 = 2, referee = 3;
    void Validate() const;
};
struct EthercatDeviceOptions {
    UartSensorBindings uart_sensors;
    bool strict_hi91_crc = false;
    bool decode_motors = true;
};
class EthercatSession {
   public:
    EthercatSession(const ethercat::AdapterPdoConfig& capacity,
                    const ethercat::AdapterPdoLayout& layout,
                    EthercatDeviceOptions options,
                    const devices::MotorConfig& motors = {},
                    const devices::ImuConfig& imu = {});
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
    const devices::MotorFeedbackDecoder& motor_decoder() const {
        return motors_;
    }
    const devices::WflySbusDecoder& wfly_sbus_decoder() const {
        return remote_;
    }
    const devices::Hi91Decoder& hi91_decoder() const {
        return imu_;
    }
    const devices::RefereeDecoder& referee_decoder() const {
        return referee_;
    }
    const devices::OnboardImu& onboard_imu() const {
        return onboard_imu_;
    }
    UartReceiveQueue& uart_queue() {
        return uart_queue_;
    }
    std::array<uint8_t, 28> diagnostics() const {
        return diagnostics_.Read();
    }
    struct StatusReply {
        uint32_t sequence = 0;
        int32_t result = 0;
        std::array<uint32_t, 3> values{};
    };
    // target: 0=session, 1=CAN, 2=UART; page 0..63, channel is logical.
    uint32_t RequestStatus(uint8_t target, uint8_t channel, uint8_t page = 0);
    StatusReply status_reply() const {
        return status_reply_.Read();
    }
    uint32_t boot_id() const {
        return boot_id_;
    }  // Master thread only.
    bool active() const {
        return active_;
    }  // Master thread only.
    // Called only by the EtherCAT master thread. No bus access here.
    bool Receive(std::span<const uint8_t> image, uint64_t now_ns,
                 uint64_t session_id);
    bool Encode(std::span<uint8_t> image, uint64_t now_ns, uint64_t session_id,
                uint32_t exchange, uint32_t flags);
    void Activate(uint64_t now_ns) {
        transmitter_.Activate(now_ns);
    }
    void Fail(const std::string& reason) {
        transmitter_.Fail(reason);
    }

   private:
    bool FailInput(const char* reason);
    void Discontinuity(unsigned port);
    const ethercat::AdapterPdoConfig capacity_;
    const ethercat::AdapterPdoLayout layout_;
    const EthercatDeviceOptions options_;
    EthercatTransmitter transmitter_;
    devices::MotorFeedbackDecoder motors_;
    devices::WflySbusDecoder remote_;
    devices::Hi91Decoder imu_;
    devices::RefereeDecoder referee_;
    devices::OnboardImu onboard_imu_;
    UartReceiveQueue uart_queue_;
    concurrency::PublishedSnapshot<std::array<uint8_t, 28>> diagnostics_;
    std::array<std::optional<uint32_t>, 2> sources_{};
    std::array<std::optional<uint32_t>, 4> can_{};
    std::array<std::optional<uint32_t>, 6> uart_rx_{}, uart_tx_{};
    std::optional<uint32_t> imu_offset_;
    uint32_t diagnostics_offset_ = 0;
    mutable concurrency::PriorityMutex status_mutex_;
    uint32_t status_request_ = 0;
    uint8_t status_target_ = 0, status_channel_ = 0;
    concurrency::PublishedSnapshot<StatusReply> status_reply_;
    uint32_t boot_id_ = 0;
    uint64_t last_publish_ns_ = 0;
    uint32_t publish_sequence_ = 0, imu_sequence_ = 0;
    bool have_publish_ = false, have_imu_ = false, active_ = false;
    struct Sequence {
        uint32_t value = 0;
        bool present = false;
    };
    struct CanLatest {
        uint32_t id = 0;
        Sequence sequence{};
        std::array<uint8_t, 19> frame{};
    };
    std::array<Sequence, 4> can_sequence_{};
    std::array<std::array<CanLatest, 32>, 4> can_latest_{};
    std::array<uint32_t, 4> can_drops_{};
    std::array<Sequence, 6> uart_sequence_{};
    std::array<uint32_t, 6> uart_peer_ack_{};
    std::array<bool, 6> uart_peer_ack_valid_{};
};
}  // namespace dust_control::host
