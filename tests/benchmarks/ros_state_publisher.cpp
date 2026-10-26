/* SPDX-License-Identifier: Apache-2.0 */
#include <chrono>
#include <cstdint>
#include <dust_control/msg/chassis_telemetry.hpp>
#include <rclcpp/rclcpp.hpp>

// 仅用于通讯基准：忙等待维持 1 kHz，隔离虚拟机休眠唤醒延迟；不连接硬件。
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dust_state_benchmark");
    auto publisher =
        node->create_publisher<dust_control::msg::ChassisTelemetry>(
            "/dust_control/chassis/telemetry", rclcpp::QoS(rclcpp::KeepLast(1))
                                                   .best_effort()
                                                   .durability_volatile());
    dust_control::msg::ChassisTelemetry message;
    message.header.frame_id = "base_link";
    message.simulated = true;
    message.source_connected = true;
    const auto period = std::chrono::milliseconds(1);
    auto release = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        release += period;
        // 忙等待仅存在于此独立测试进程，不能替代生产控制线程的周期等待。
        while (rclcpp::ok() && std::chrono::steady_clock::now() < release) {
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - release >= period) {
            const auto missed = (now - release) / period;
            release += missed * period;
            message.missed_releases += missed;
        }
        message.monotonic_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch())
                .count();
        message.header.stamp = node->now();
        ++message.sequence;
        if (rclcpp::ok()) {
            publisher->publish(message);
        }
    }
    rclcpp::shutdown();
    return 0;
}
