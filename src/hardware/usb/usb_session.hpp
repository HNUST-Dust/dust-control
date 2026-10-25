/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "usb_link.hpp"
#include "usb_transmitter.hpp"

namespace wbr::host {
// 拥有单个 USB 会话的收发资源；业务线程先注册命令组，退出后再停止会话。
class UsbSession {
   public:
    UsbSession(const UsbOptions& options,
               const dust::devices::MotorConfig& motors,
               const dust::devices::ImuConfig& imu);
    UsbSession(const UsbOptions& options,
               const dust::devices::MotorConfig& motors,
               const dust::devices::ImuConfig& imu,
               UsbTransmitOptions transmit);
    ~UsbSession();
    bool Start();
    void Stop();
    bool healthy() const {
        return link_.healthy() && transmitter_.healthy();
    }
    std::string error();
    const UsbLink& link() const {
        return link_;
    }
    const UsbTransmitter& transmitter() const {
        return transmitter_;
    }
    UsbTransmitter& transmitter() {
        return transmitter_;
    }

   private:
    UartReceiveQueue uart_queue_;
    UsbLink link_;
    UsbTransmitter transmitter_;
    bool started_ = false;
    bool stopped_ = false;
};
}  // namespace wbr::host
