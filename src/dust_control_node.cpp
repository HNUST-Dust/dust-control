/* SPDX-License-Identifier: Apache-2.0 */
#include "dust_control_node.hpp"

#include <chassis/runtime/realtime_loop.hpp>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace dust::ros {
namespace {
constexpr double kStandardGravityMps2 = 9.80665;
constexpr auto kPublicationPeriod = std::chrono::milliseconds(20);
}  // namespace

DustControlNode::DustControlNode(const rclcpp::NodeOptions& options)
    : Node("dust_control", options), options_(DeclareStartupOptions()) {
    // 所有 ROS 实体先于实时控制线程创建；回调不执行 USB 或电机操作。
    robot_ = std::make_unique<app::Robot>(options_);
    velocity_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        "~/cmd_vel", rclcpp::QoS(1),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
            ReceiveVelocityCommand(*msg);
        });
    motion_subscription_ =
        create_subscription<dust_control::msg::MotionCommand>(
            "~/motion_command", rclcpp::QoS(1),
            [this](dust_control::msg::MotionCommand::ConstSharedPtr msg) {
                ReceiveMotionCommand(*msg);
            });
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        "~/imu", rclcpp::SensorDataQoS());
    joints_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
        "~/joint_states", rclcpp::SensorDataQoS());
    status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "~/status", rclcpp::QoS(1));
    publication_timer_ = create_wall_timer(kPublicationPeriod,
                                           [this] { PublishObservations(); });
    lifecycle_timer_ = create_wall_timer(std::chrono::milliseconds(5),
                                         [this] { CheckModuleStatus(); });
    robot_->Start();
}

app::Options DustControlNode::DeclareStartupOptions() {
    app::Options options;
    rcl_interfaces::msg::ParameterDescriptor startup;
    startup.read_only = true;
    startup.description = "Startup-only control configuration";
    options.simulate = declare_parameter("simulate", false, startup);
    options.enable_output = declare_parameter("enable_output", false, startup);
    options.usb.realtime = declare_parameter("realtime", true, startup);
    options.control_cpu = declare_parameter("control_cpu", 3, startup);
    options.usb.cpu = declare_parameter("usb_cpu", 3, startup);
    options.usb.priority = declare_parameter("priority", 80, startup);
    options.usb.strict_hi91_crc =
        declare_parameter("strict_hi91_crc", false, startup);
    options.config_directory =
        declare_parameter<std::string>("config_directory", "", startup);
    options.usb.serial =
        declare_parameter<std::string>("usb_serial", "", startup);
    options.usb.product =
        declare_parameter<std::string>("usb_product", "dust-hpm6750", startup);
    const auto pid = declare_parameter<int>("usb_pid", -1, startup);
    if (pid < -1 || pid > 65535) {
        throw std::runtime_error("usb_pid must be -1 or 0..65535");
    }
    if (pid >= 0) {
        options.usb.pid = uint16_t(pid);
    }
    options.seconds = declare_parameter("run_seconds", 0.0, startup);
    options.foxglove_port = declare_parameter("foxglove_port", 0, startup);
    const auto source =
        declare_parameter<std::string>("command_source", "remote", startup);
    if (source != "remote" && source != "ros") {
        throw std::runtime_error("command_source must be remote or ros");
    }
    options.ros_commands = source == "ros";
    const auto timeout_ms =
        declare_parameter("command_timeout_ms", 200, startup);
    if (timeout_ms <= 0 || timeout_ms > 10000) {
        throw std::runtime_error("command_timeout_ms must be 1..10000");
    }
    options.command_timeout_ns = uint64_t(timeout_ms) * 1000000;
    return options;
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

void DustControlNode::ReceiveVelocityCommand(
    const geometry_msgs::msg::Twist& command) {
    if (command.linear.y != 0 || command.linear.z != 0 ||
        command.angular.x != 0 || command.angular.y != 0) {
        ++rejected_command_count_;
        return;
    }
    SubmitMotionCommand(command.linear.x, command.angular.z, 0);
}

void DustControlNode::ReceiveMotionCommand(
    const dust_control::msg::MotionCommand& command) {
    if (!command.header.frame_id.empty() &&
        command.header.frame_id != "base_link") {
        ++rejected_command_count_;
        return;
    }
    SubmitMotionCommand(command.forward_velocity_m_s, command.yaw_rate_rad_s,
                        command.leg_length_rate_m_s);
}

void DustControlNode::SubmitMotionCommand(double forward, double yaw,
                                          double leg_rate) {
    if (!options_.ros_commands ||
        !robot_->chassis().commands().Push(
            {forward, yaw, leg_rate, wbr::realtime::MonotonicTimeNs()})) {
        ++rejected_command_count_;
    }
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
    if (robot_->finished()) {
        return;
    }
    wbr::host::TelemetrySample sample;
    bool available = false;
    while (robot_->chassis().TakeObservation(sample)) {
        available = true;
    }
    if (!available) {
        return;
    }
    const auto stamp = now();
    PublishStatusReport(sample, stamp);
    PublishImuSample(sample.input, stamp);
    PublishJointStates(sample.input, stamp);
}

void DustControlNode::PublishStatusReport(
    const wbr::host::TelemetrySample& sample, const rclcpp::Time& stamp) {
    diagnostic_msgs::msg::DiagnosticArray status;
    status.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus diagnostic;
    diagnostic.name = get_fully_qualified_name();
    diagnostic.hardware_id =
        options_.simulate ? "simulation" : options_.usb.product;
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
    append_value("command_source", options_.ros_commands ? "ros" : "remote");
    append_value("rejected_commands", std::to_string(rejected_command_count_));
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

void DustControlNode::PublishImuSample(const modules::ChassisCycleInput& input,
                                       const rclcpp::Time& stamp) {
    if (input.imu_fresh && input.imu.sequence != last_imu_sequence_) {
        const auto monotonic_us = wbr::realtime::MonotonicTimeNs() / 1000;
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
    const modules::ChassisCycleInput& input, const rclcpp::Time& stamp) {
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
}  // namespace dust::ros
