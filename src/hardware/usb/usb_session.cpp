/* SPDX-License-Identifier: Apache-2.0 */
#include "usb_session.hpp"

#include <algorithm>
#include <stdexcept>

namespace wbr::host {
UsbSession::UsbSession(const UsbOptions& options,
                       const dust::devices::MotorConfig& motors,
                       const dust::devices::ImuConfig& imu)
    : UsbSession(options, motors, imu,
                 {options.realtime, options.cpu,
                  options.priority ? std::max(1, options.priority - 1) : 0}) {}
UsbSession::UsbSession(const UsbOptions& options,
                       const dust::devices::MotorConfig& motors,
                       const dust::devices::ImuConfig& imu,
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
}  // namespace wbr::host
