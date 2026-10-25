/* SPDX-License-Identifier: Apache-2.0 */
#include "dust_control_node.hpp"

#include <chassis/runtime/realtime_loop.hpp>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "chassis_telemetry.hpp"

namespace dust_control::ros {
namespace {
constexpr double kStandardGravityMps2 = 9.80665;
constexpr auto kPublicationPeriod = std::chrono::milliseconds(1);
}  // namespace

DustControlNode::DustControlNode(const rclcpp::NodeOptions& options)
    : Node("dust_control", options) {
    // 所有 ROS 实体先于实时控制线程创建；回调不执行 USB 或电机操作。
    robot_ = std::make_unique<app::Robot>(DeclareConfigurationDirectory());
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        "~/imu", rclcpp::SensorDataQoS());
    joints_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
        "~/joint_states", rclcpp::SensorDataQoS());
    status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "~/status", rclcpp::QoS(1));
    telemetry_publisher_ =
        create_publisher<dust_control::msg::ChassisTelemetry>(
            "~/chassis/telemetry", rclcpp::QoS(rclcpp::KeepLast(1))
                                       .best_effort()
                                       .durability_volatile());
    publication_timer_ = create_wall_timer(kPublicationPeriod,
                                           [this] { PublishObservations(); });
    auxiliary_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] {
        PublishAuxiliaryObservations();
    });
    lifecycle_timer_ = create_wall_timer(std::chrono::milliseconds(5),
                                         [this] { CheckModuleStatus(); });
    robot_->Start();
}

std::string DustControlNode::DeclareConfigurationDirectory() {
    const auto& overrides =
        get_node_parameters_interface()->get_parameter_overrides();
    for (const auto* legacy :
         {"simulate", "enable_output", "realtime", "run_seconds",
          "command_source", "command_timeout_ms", "control_cpu", "usb_cpu",
          "priority", "strict_hi91_crc", "usb_serial", "usb_product",
          "usb_pid"}) {
        if (overrides.contains(legacy)) {
            throw std::invalid_argument(
                std::string("Unsupported startup parameter: ") + legacy +
                "; use config_directory and robot.yaml");
        }
    }
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description =
        "Directory containing robot.yaml and module configuration";
    const auto directory =
        declare_parameter<std::string>("config_directory", "", descriptor);
    if (directory.empty()) {
        throw std::invalid_argument("Missing config_directory");
    }
    return directory;
}

DustControlNode::~DustControlNode() {
    Stop();
}

void DustControlNode::Stop() {
    if (robot_) {
        robot_->Stop();
    }
}

std::string DustControlNode::runtime_error() const {
    return robot_ && robot_->finished() ? robot_->error() : std::string{};
}

void DustControlNode::CheckModuleStatus() {
    if (robot_->finished()) {
        if (!robot_->error().empty()) {
            RCLCPP_ERROR(get_logger(), "%s", robot_->error().c_str());
        }
        rclcpp::shutdown(get_node_base_interface()->get_context());
    }
}

void DustControlNode::PublishObservations() {
    if (robot_->finished() || !robot_->configuration().chassis.enabled) {
        return;
    }
    chassis::ChassisObservation sample;
    bool available = false;
    // 限定消费次数，防止生产者持续写入时占满 ROS executor。
    for (size_t count = 0;
         count < chassis::ChassisObservationQueue::kCapacity &&
         robot_->chassis().TakeObservation(sample);
         ++count) {
        available = true;
        latest_observation_ = sample;
        have_observation_ = true;
    }
    // 空队列不重复发布新鲜样本；源停更时继续告知观测已经失效。
    const auto monotonic_ns = dust_control::realtime::MonotonicTimeNs();
    if (have_observation_ &&
        (available ||
         monotonic_ns - latest_observation_.monotonic_ns > 200000000)) {
        telemetry_publisher_->publish(
            MakeChassisTelemetry(latest_observation_, now(), monotonic_ns));
    }
}

void DustControlNode::PublishAuxiliaryObservations() {
    if (robot_->finished() || !have_observation_) {
        return;
    }
    const auto stamp = now();
    const auto monotonic_ns = dust_control::realtime::MonotonicTimeNs();
    if (monotonic_ns - last_status_ns_ >= 20000000) {
        PublishStatusReport(latest_observation_, stamp);
        last_status_ns_ = monotonic_ns;
    }
    PublishImuSample(latest_observation_.input, stamp);
    PublishJointStates(latest_observation_.input, stamp);
}

void DustControlNode::PublishStatusReport(
    const chassis::ChassisObservation& sample, const rclcpp::Time& stamp) {
    diagnostic_msgs::msg::DiagnosticArray status;
    status.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus diagnostic;
    diagnostic.name = get_fully_qualified_name();
    diagnostic.hardware_id =
        robot_->configuration().execution.simulate
            ? "simulation"
            : robot_->configuration()
                  .adapters.at(robot_->configuration().chassis.adapter)
                  .usb.product;
    diagnostic.level = sample.input.feedback_valid && sample.input.imu_fresh &&
                               sample.input.remote_fresh
                           ? diagnostic_msgs::msg::DiagnosticStatus::OK
                           : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diagnostic.message =
        sample.simulated ? "offline simulation" : "control running";
    const auto append_value = [&diagnostic](const std::string& key,
                                            const std::string& text) {
        diagnostic_msgs::msg::KeyValue entry;
        entry.key = key;
        entry.value = text;
        diagnostic.values.push_back(std::move(entry));
    };
    append_value("control_state", std::to_string(unsigned(sample.state)));
    append_value("output_requested",
                 sample.input.requested_enable ? "true" : "false");
    append_value("motor_stale_mask",
                 std::to_string(sample.input.motor_stale_mask));
    append_value("imu_fresh", sample.input.imu_fresh ? "true" : "false");
    append_value("remote_fresh", sample.input.remote_fresh ? "true" : "false");
    append_value("missed_releases", std::to_string(sample.missed_releases));
    append_value("control_duration_ns",
                 std::to_string(sample.control_duration_ns));
    status.status.push_back(std::move(diagnostic));
    status_publisher_->publish(status);
}

void DustControlNode::PublishImuSample(
    const dust_control::modules::ChassisCycleInput& input,
    const rclcpp::Time& stamp) {
    if (input.imu_fresh && input.imu.sequence != last_imu_sequence_) {
        const auto monotonic_us =
            dust_control::realtime::MonotonicTimeNs() / 1000;
        const auto sample_us = input.imu.precise_timestamp_us;
        const auto age_ns = monotonic_us >= sample_us
                                ? (monotonic_us - sample_us) * 1000
                                : UINT64_MAX;
        // ROS 仿真时钟尚未启动或回退到零时，不构造负时间戳。
        if (age_ns <= uint64_t(stamp.nanoseconds())) {
            sensor_msgs::msg::Imu imu;
            imu.header.stamp =
                stamp - rclcpp::Duration::from_nanoseconds(int64_t(age_ns));
            imu.header.frame_id = "hi91_link";
            // 未提供验证过的 HI91 到 base_link 变换，不发布机体姿态四元数。
            imu.orientation_covariance[0] = -1;
            imu.angular_velocity.x = input.imu.pitch_rate_rad_s;
            imu.angular_velocity.y = input.imu.roll_rate_rad_s;
            imu.angular_velocity.z = input.imu.yaw_rate_rad_s;
            imu.linear_acceleration.x =
                input.imu.accel_g[0] * kStandardGravityMps2;
            imu.linear_acceleration.y =
                input.imu.accel_g[1] * kStandardGravityMps2;
            imu.linear_acceleration.z =
                input.imu.accel_g[2] * kStandardGravityMps2;
            imu_publisher_->publish(imu);
            last_imu_sequence_ = input.imu.sequence;
        }
    }
}

void DustControlNode::PublishJointStates(
    const dust_control::modules::ChassisCycleInput& input,
    const rclcpp::Time& stamp) {
    if (input.feedback_valid) {
        sensor_msgs::msg::JointState joints;
        joints.header.stamp = stamp;
        joints.name = {"left_joint_b", "left_joint_d", "right_joint_b",
                       "right_joint_d"};
        joints.position = {
            input.joint_position.left.b, input.joint_position.left.d,
            input.joint_position.right.b, input.joint_position.right.d};
        joints.velocity = {
            input.joint_velocity.left.b, input.joint_velocity.left.d,
            input.joint_velocity.right.b, input.joint_velocity.right.d};
        joints.effort = {input.joint_torque_feedback.left.b,
                         input.joint_torque_feedback.left.d,
                         input.joint_torque_feedback.right.b,
                         input.joint_torque_feedback.right.d};
        joints_publisher_->publish(joints);
    }
}
}  // namespace dust_control::ros
