"""Start one chassis node; outputs remain disabled unless explicitly requested."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("simulate", default_value="false"),
        DeclareLaunchArgument("realtime", default_value="true"),
        DeclareLaunchArgument("enable_output", default_value="false"),
        DeclareLaunchArgument("command_source", default_value="remote"),
        DeclareLaunchArgument("usb_serial", default_value=""),
        DeclareLaunchArgument("run_seconds", default_value="0.0"),
    ]
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=bool)
        for name in ("simulate", "realtime", "enable_output")
    }
    parameters.update({
        "command_source": ParameterValue(LaunchConfiguration("command_source"), value_type=str),
        "usb_serial": ParameterValue(LaunchConfiguration("usb_serial"), value_type=str),
        "run_seconds": ParameterValue(LaunchConfiguration("run_seconds"), value_type=float),
        "config_directory": PathJoinSubstitution([FindPackageShare("dust_control"), "config"]),
    })
    return LaunchDescription(arguments + [Node(
        package="dust_control", executable="dust_control_node", name="dust_control",
        output="screen", parameters=[parameters],
    )])
