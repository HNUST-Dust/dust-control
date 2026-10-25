/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "chassis/controller_inputs.h"
#include "hardware/usb/usb_link.hpp"
#include "motor_fixture.hpp"

namespace dust::test {
// 只在测试中驱动真实接收入口；不打开 USB，不复制协议解析或路由逻辑。
class UsbReportFixture {
   public:
    explicit UsbReportFixture(
        wbr::host::UartReceiveQueue& store, bool strict_crc = false,
        const dust::devices::MotorConfig& motors = MotorFixture(),
        const dust::devices::ImuConfig& imu = {})
        : link_(store, Options(strict_crc), motors, imu) {}
    UsbReportFixture(wbr::host::UartReceiveQueue& store,
                     const wbr::host::UsbOptions& options)
        : link_(store, options, MotorFixture()) {}
    bool Consume(const wbr::bridge::UsbInPacket& packet, uint64_t receive_us) {
        return link_.ReceiveSensorReport(packet, receive_us);
    }
    void Reset() {
        link_.ResetSensors();
    }
    void PollRemoteTimeout(uint64_t now_us) {
        link_.sbus_decoder_.PollTimeout(now_us);
    }
    bool remote_timeout_pending() const {
        return link_.sbus_decoder_.timeout_pending();
    }

    struct Snapshot {
        modules::ControllerInputs control{};
        dust::devices::OnboardImuSample onboard{};
        dust::devices::RefereeSample referee{};
    };
    // 测试便捷视图；生产控制循环直接读取所需设备，不存在集中发布对象。
    Snapshot Read() const {
        Snapshot snapshot;
        snapshot.control.motors = link_.motor_decoder().Read();
        const auto remote = link_.sbus_decoder().Read();
        snapshot.control.remote = remote.remote;
        snapshot.control.remote_disable_pose = remote.disable_pose;
        snapshot.control.imu = link_.hi91_decoder().Read();
        snapshot.onboard = link_.onboard_imu().Read();
        snapshot.referee = link_.referee_decoder().Read();
        return snapshot;
    }

   private:
    static wbr::host::UsbOptions Options(bool strict_crc) {
        wbr::host::UsbOptions options;
        options.realtime = false;
        options.strict_hi91_crc = strict_crc;
        return options;
    }
    wbr::host::UsbLink link_;
};
}  // namespace dust::test
