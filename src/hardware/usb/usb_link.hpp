/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <libusb.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>

#include "bridge_protocol.hpp"
#include "hardware/can_frame.hpp"
#include "hardware/imu/hi91_decoder.hpp"
#include "hardware/imu/onboard_imu.hpp"
#include "hardware/motors/motor_protocol.hpp"
#include "hardware/referee/referee_decoder.hpp"
#include "hardware/remote/sbus_decoder.hpp"
#include "uart_receive_queue.hpp"

namespace dust::test {
class UsbReportFixture;
}

namespace wbr::host {
// 每块 Adapter 独立绑定；0 表示不绑定，1..6 对应物理 UART。
struct UartSensorBindings {
    uint8_t remote = 1;
    uint8_t hi91 = 2;
    uint8_t referee = 3;
    void Validate() const {
        std::array<bool, 7> occupied{};
        for (const auto channel : {remote, hi91, referee}) {
            if (channel > 6 || (channel && occupied[channel])) {
                throw std::invalid_argument(
                    "UART sensor bindings require unique channels in 1..6 (0 "
                    "disables)");
            }
            occupied[channel] = channel != 0;
        }
    }
};
struct UsbOptions {
    UartSensorBindings uart_sensors;

    uint16_t vid = 0x34b7;
    std::optional<uint16_t> pid;  // 配置后执行精确筛选，包括 0 和 0xffff。
    std::string manufacturer = "Dust";
    std::string product = "dust-hpm6750";
    std::string serial;
    bool realtime = true;
    int cpu = 3;
    int priority = 80;
    bool strict_hi91_crc = false;
    bool decode_motors = true;
    std::string thread_name = "dust-usb-rx";
};

class UsbLink {
   public:
    UsbLink(UartReceiveQueue& uart_queue, const UsbOptions& options,
            const dust::devices::MotorConfig& motors = {},
            const dust::devices::ImuConfig& imu = {});
    ~UsbLink();
    // 阻塞等待设备发现、v4 握手、安全配置和 Arm 确认；失败后由 Close 清理。
    bool Open();
    // 阻塞等待事件线程取消并排空传输，再释放 USB 资源。
    void Close();
    bool healthy() const;
    std::string error();
    uint32_t session() const {
        return session_;
    }

    uint32_t capabilities() const {
        return capabilities_;
    }

    // 引用寿命不超过 UsbLink；控制线程只调用各设备的 Read() 获取副本。
    const dust::devices::MotorFeedbackDecoder& motor_decoder() const {
        return motor_decoder_;
    }
    const dust::devices::SbusDecoder& sbus_decoder() const {
        return sbus_decoder_;
    }
    const dust::devices::Hi91Decoder& hi91_decoder() const {
        return hi91_decoder_;
    }
    const dust::devices::RefereeDecoder& referee_decoder() const {
        return referee_decoder_;
    }
    const dust::devices::OnboardImu& onboard_imu() const {
        return onboard_imu_;
    }

   private:
    friend class dust::test::UsbReportFixture;
    friend class UsbTransmitter;
    // 握手完成后，仅发送线程推进命令序号和提交业务传输。
    bool Send(std::span<const dust::devices::CanFrame> frames);
    bool SendUart(uint8_t channel, const uint8_t* bytes, size_t size);
    void Stop();
    unsigned FreeTransmitSlots() const;

    struct UartSequence {
        uint32_t sequence = 0;
        bool present = false;
    };

    struct ReceiveTransfer {
        UsbLink* owner = nullptr;
        libusb_transfer* transfer = nullptr;
        bridge::UsbInPacket data{};
        bool pending = false;
    };

    struct TransmitTransfer {
        UsbLink* owner = nullptr;
        libusb_transfer* transfer = nullptr;
        bridge::UsbOutRequest data{};
        std::atomic<bool> free{true};
    };

    // 调用线程拥有发现/握手流程；事件线程只完成传输和发布会话状态。
    bool OpenDevice();
    bool StartSession();
    bool ConfigureSafetyAndArm();
    bool ExchangeControlRequest(bridge::UsbOutRequest request);
    bool FailSession(const std::string& reason);

    // 接收提交与回调成对排列；发送入队与回调同理。
    void SubmitReceive(ReceiveTransfer& transfer);
    static void LIBUSB_CALL CompleteReceive(libusb_transfer* transfer);
    bool QueueTransmit(const uint8_t* bytes, size_t size);
    static void LIBUSB_CALL CompleteTransmit(libusb_transfer* transfer);
    void RunEventLoop();

    // 以下操作由接收线程独占，各 Decoder 自行发布线程安全快照。
    bool ReceiveSensorReport(const bridge::UsbInPacket& report,
                             uint64_t arrival_us);
    void ReceiveCanReport(const bridge::CanReceiveReport& report,
                          uint64_t arrival_us);
    bool ReceiveUartReport(const bridge::UartReceiveReport& report,
                           uint64_t arrival_us);
    bool ReceiveOnboardImuReport(const bridge::ImuSampleReport& report,
                                 uint64_t arrival_us);
    void ResetSensors();

    UsbOptions options_;
    const dust::devices::MotorConfig motor_config_;
    UartReceiveQueue& uart_queue_;
    dust::devices::MotorFeedbackDecoder motor_decoder_;
    dust::devices::SbusDecoder sbus_decoder_;
    dust::devices::Hi91Decoder hi91_decoder_;
    dust::devices::RefereeDecoder referee_decoder_;
    dust::devices::OnboardImu onboard_imu_;
    std::array<UartSequence, bridge::kUartPortCount> uart_sequences_{};
    libusb_context* context_ = nullptr;
    libusb_device_handle* device_ = nullptr;
    bool claimed_ = false;
    std::array<ReceiveTransfer, 4> receive_transfers_{};
    std::array<TransmitTransfer, 8> transmit_transfers_{};
    std::thread thread_;
    // 跨线程状态由原子量发布，错误文本与条件等待由 mutex_ 保护。
    std::atomic<bool> closing_{false};
    std::atomic<bool> failed_{false};
    std::atomic<bool> ready_received_{false};
    std::atomic<unsigned> active_transfers_{0};
    std::atomic<uint64_t> last_receive_us_{0};
    std::atomic<bool> receive_preposted_{false};
    // 会话启动后，命令序列由发送线程独占推进。
    uint32_t session_ = 0;
    uint32_t request_sequence_ = 0;
    uint32_t capabilities_ = 0;
    uint32_t max_lease_us_ = 0;
    std::atomic<uint32_t> awaited_request_sequence_{0};
    std::atomic<uint32_t> acknowledged_request_sequence_{0};
    std::atomic<int32_t> result_code_{0};
    std::atomic<uint8_t> awaited_request_kind_{0};
    std::mutex mutex_;
    std::condition_variable ready_cv_;
    std::string error_;
};
}  // namespace wbr::host
