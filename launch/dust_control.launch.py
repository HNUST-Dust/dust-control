"""Load the dust-control deployment and execution settings from its configuration directory."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_directory",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("dust_control"), "config"]
                ),
            ),
            Node(
                package="dust_control",
                executable="dust_control_node",
                name="dust_control",
                output="screen",
                parameters=[
                    {"config_directory": LaunchConfiguration("config_directory")}
                ],
            ),
        ]
    )
