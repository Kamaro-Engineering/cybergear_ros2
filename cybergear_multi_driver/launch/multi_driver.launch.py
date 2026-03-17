"""Launch the cybergear_multi_driver node with a configurable parameter file."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("cybergear_multi_driver")

    default_params = PathJoinSubstitution(
        [pkg_share, "config", "default_params.yaml"]
    )

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Absolute path to the ROS 2 parameter file for the driver node.",
    )

    node_name_arg = DeclareLaunchArgument(
        "node_name",
        default_value="cybergear_multi_driver",
        description="ROS 2 node name.",
    )

    driver_node = Node(
        package="cybergear_multi_driver",
        executable="cybergear_multi_driver_node",
        name=LaunchConfiguration("node_name"),
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        params_arg,
        node_name_arg,
        driver_node,
    ])
