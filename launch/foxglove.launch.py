"""Start once per ROS graph; business nodes have independent lifecycles."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8765"),
        DeclareLaunchArgument("address", default_value="127.0.0.1"),
        Node(
            package="foxglove_bridge", executable="foxglove_bridge",
            name="foxglove_bridge", output="screen",
            parameters=[{
                "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                "address": ParameterValue(LaunchConfiguration("address"), value_type=str),
                # 统一显示话题；不通过可视化入口发布控制命令或修改参数。
                "capabilities": ["connectionGraph"],
                "min_qos_depth": 1,
                "max_qos_depth": 1,
                "best_effort_qos_topic_whitelist": [".*/chassis/telemetry"],
            }],
        ),
    ])
