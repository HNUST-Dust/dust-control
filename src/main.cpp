/* SPDX-License-Identifier: Apache-2.0 */
#include <dust_control_node.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<dust_control::ros::DustControlNode>();
        rclcpp::spin(node);
        node->Stop();
        const auto error = node->runtime_error();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        if (!error.empty()) {
            std::cerr << error << std::endl;
            return 1;
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 1;
    }
}
