"""Start the bounded frontier explorer; mapping and Nav2 must already be running."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.normpath(os.path.join(
        os.path.dirname(__file__), "..", "config", "frontier_exploration.yaml"))
    return LaunchDescription([
        DeclareLaunchArgument("center_override_enabled", default_value="false"),
        DeclareLaunchArgument("center_x", default_value="0.0"),
        DeclareLaunchArgument("center_y", default_value="0.0"),
        Node(
            package="rescue_robot_bringup",
            executable="frontier_exploration.py",
            name="frontier_exploration",
            output="screen",
            parameters=[config, {
                "center_override_enabled": ParameterValue(
                    LaunchConfiguration("center_override_enabled"), value_type=bool),
                "center_x": ParameterValue(LaunchConfiguration("center_x"), value_type=float),
                "center_y": ParameterValue(LaunchConfiguration("center_y"), value_type=float),
            }],
        ),
    ])
