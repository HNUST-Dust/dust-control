/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <dust_control/msg/motion_command.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

#include "robot.hpp"

namespace dust::ros {
class DustControlNode : public rclcpp::Node {
   public:
    explicit DustControlNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~DustControlNode() override;
    // 调用线程设置退出请求并阻塞等待控制线程完成停止；不在控制线程调用。
    void Stop();
    std::string runtime_error() const;

   private:
    app::Options DeclareStartupOptions();
    void ReceiveVelocityCommand(const geometry_msgs::msg::Twist& command);
    void ReceiveMotionCommand(const dust_control::msg::MotionCommand& command);
    void SubmitMotionCommand(double forward_m_s, double yaw_rad_s,
                             double leg_rate_m_s);

    void CheckModuleStatus();
    void PublishObservations();
    void PublishStatusReport(const wbr::host::TelemetrySample& sample,
                             const rclcpp::Time& stamp);
    void PublishImuSample(const modules::ChassisCycleInput& input,
                          const rclcpp::Time& stamp);
    void PublishJointStates(const modules::ChassisCycleInput& input,
                            const rclcpp::Time& stamp);

    app::Options options_;
    std::unique_ptr<app::Robot> robot_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        velocity_subscription_;
    rclcpp::Subscription<dust_control::msg::MotionCommand>::SharedPtr
        motion_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        joints_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        status_publisher_;
    rclcpp::TimerBase::SharedPtr publication_timer_;
    rclcpp::TimerBase::SharedPtr lifecycle_timer_;
    // 默认互斥回调组独占订阅、发布和以下统计，不由控制线程修改。
    uint32_t last_imu_sequence_ = 0;
    uint64_t rejected_command_count_ = 0;
};
}  // namespace dust::ros
