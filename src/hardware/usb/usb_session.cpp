/* SPDX-License-Identifier: Apache-2.0 */
#include "usb_session.hpp"

#include <algorithm>
#include <chrono>
#include <random>
#include <stdexcept>

namespace dust_control::host {
UsbSession::UsbSession(const UsbOptions& options,
                       const dust_control::devices::MotorConfig& motors,
                       const dust_control::devices::ImuConfig& imu)
    : UsbSession(options, motors, imu,
                 {options.realtime, options.cpu,
                  options.priority ? std::max(1, options.priority - 1) : 0}) {}
UsbSession::UsbSession(const UsbOptions& options,
                       const dust_control::devices::MotorConfig& motors,
                       const dust_control::devices::ImuConfig& imu,
                       UsbTransmitOptions transmit)
    : link_(uart_queue_, options, motors, imu), transmitter_(link_, transmit) {}
UsbSession::~UsbSession() {
    Stop();
}
bool UsbSession::Start() {
    if (started_ || stopped_) {
        throw std::logic_error("USB session cannot be restarted");
    }
    started_ = true;
    if (!link_.Open() || !transmitter_.Start()) {
        Stop();
        return false;
    }
    return true;
}
void UsbSession::Stop() {
    stopped_ = true;
    transmitter_.Stop();
    link_.Close();
}
std::string UsbSession::error() {
    const auto link_error = link_.error();
    return link_error.empty() ? transmitter_.error() : link_error;
}

// UsbLink 拥有握手状态；UsbSession 负责整个会话资源的启动和停止顺序。
bool UsbLink::StartSession() {
    for (auto& input : receive_transfers_) {
        input.owner = this;
        input.transfer = libusb_alloc_transfer(0);
        if (!input.transfer) {
            return FailSession("USB IN allocation");
        }
    }

    for (auto& output : transmit_transfers_) {
        output.owner = this;
        output.transfer = libusb_alloc_transfer(0);
        if (!output.transfer) {
            return FailSession("USB OUT allocation");
        }
    }

    session_ = std::random_device{}();
    if (!session_) {
        session_ = 1;
    }

    thread_ = std::thread([this] { RunEventLoop(); });
    // 发送 Hello 使 MCU 开始发布前，必须先启动接收并完成实时配置和 mlockall；
    // 任一步骤失败都不能启动会话。
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!ready_cv_.wait_for(lock, std::chrono::seconds(2), [this] {
                return receive_preposted_.load() || failed_.load();
            })) {
            lock.unlock();
            return FailSession("USB IN prepost timed out");
        }
        if (failed_) {
            return false;
        }
    }

    dust_control::wire::UsbOutRequest hello{};
    dust_control::wire::Encode(dust_control::wire::HelloRequest{{0, session_}},
                               hello);
    if (!QueueTransmit(hello.data(), hello.size())) {
        return false;
    }

    std::unique_lock<std::mutex> lock(mutex_);
    if (!ready_cv_.wait_for(lock, std::chrono::seconds(2), [this] {
            return ready_received_.load() || failed_.load();
        })) {
        lock.unlock();
        return FailSession(
            "Adapter v4 handshake timed out; matching generic firmware is "
            "required");
    }

    lock.unlock();
    return !failed_ && ConfigureSafetyAndArm();
}

bool UsbLink::ExchangeControlRequest(
    dust_control::wire::UsbOutRequest request) {
    const dust_control::wire::UsbOutRequestView view(request);
    const auto sequence = view.Sequence();
    awaited_request_kind_ = uint8_t(view.MessageKind());
    awaited_request_sequence_ = sequence;
    acknowledged_request_sequence_ = 0;
    if (!QueueTransmit(request.data(), request.size())) {
        return false;
    }

    std::unique_lock<std::mutex> lock(mutex_);
    if (!ready_cv_.wait_for(lock, std::chrono::seconds(2), [this, sequence] {
            return acknowledged_request_sequence_.load() == sequence ||
                   failed_.load();
        })) {
        lock.unlock();
        return FailSession("Adapter safety configuration/arm timed out");
    }

    if (failed_) {
        return false;
    }

    if (result_code_.load()) {
        lock.unlock();
        return FailSession("Adapter rejected safety configuration/arm");
    }

    return true;
}

bool UsbLink::ConfigureSafetyAndArm() {
    const auto lease = std::min(max_lease_us_, kCommandTimeoutUs);
    dust_control::wire::ConfigureSafetyRequest safety{};
    safety.header = {++request_sequence_, session_};
    safety.lease_us = lease;
    dust_control::wire::UsbOutRequest packet{};
    if (!dust_control::wire::Encode(safety, packet)) {
        return FailSession("Invalid adapter safety lease");
    }
    if (!ExchangeControlRequest(packet)) {
        return false;
    }
    dust_control::wire::Encode(
        dust_control::wire::ArmRequest{{++request_sequence_, session_}},
        packet);
    return ExchangeControlRequest(packet);
}

void UsbLink::Stop() {
    if (!device_ || !thread_.joinable() || closing_) {
        return;
    }

    if (!failed_) {
        dust_control::wire::UsbOutRequest stop{};
        dust_control::wire::Encode(
            dust_control::wire::StopRequest{{++request_sequence_, session_}},
            stop);
        QueueTransmit(stop.data(), stop.size());
    }

    // 等待设备停止，或由 20 ms 本地看门狗触发保护。
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    ready_received_ = false;
}
}  // namespace dust_control::host
