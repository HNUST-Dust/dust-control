/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <dust_control/msg/chassis_telemetry.hpp>
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
    std::string DeclareConfigurationDirectory();

    void CheckModuleStatus();
    void PublishObservations();
    void PublishAuxiliaryObservations();
    void PublishStatusReport(const chassis::ChassisObservation& sample,
                             const rclcpp::Time& stamp);
    void PublishImuSample(const modules::ChassisCycleInput& input,
                          const rclcpp::Time& stamp);
    void PublishJointStates(const modules::ChassisCycleInput& input,
                            const rclcpp::Time& stamp);

    // 发布回调独占缓存；源停更后继续发布失效状态。
    chassis::ChassisObservation latest_observation_{};
    bool have_observation_ = false;
    rclcpp::Publisher<dust_control::msg::ChassisTelemetry>::SharedPtr
        telemetry_publisher_;
    std::unique_ptr<app::Robot> robot_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        joints_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        status_publisher_;
    rclcpp::TimerBase::SharedPtr publication_timer_;
    rclcpp::TimerBase::SharedPtr lifecycle_timer_;
    rclcpp::TimerBase::SharedPtr auxiliary_timer_;
    uint64_t last_status_ns_ = 0;
    // 默认互斥回调组独占发布和以下统计，不由控制线程修改。
    uint32_t last_imu_sequence_ = 0;
};
}  // namespace dust::ros
