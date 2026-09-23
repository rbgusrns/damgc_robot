"""Start the ROS-side survivor pipeline using the verified child launches."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package_name, launch_file, condition=None):
    launch_path = os.path.join(
        get_package_share_directory(package_name), "launch", launch_file
    )
    # The child launches reuse names such as input_topic and output_topic.
    # Isolate their defaults so one child cannot configure another by accident.
    return GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_path)
            )
        ],
        scoped=True,
        forwarding=False,
        condition=condition,
    )


def generate_launch_description():
    """Launch preprocessing, map projection, and visualization."""
    return LaunchDescription([
        DeclareLaunchArgument("enable_raw_visualizer", default_value="true"),
        _include(
            "rescue_robot_bringup", "survivor_camera_processing.launch.py"
        ),
        _include("rescue_robot_survivor", "survivor_map_transform.launch.py"),
        _include(
            "rescue_robot_survivor",
            "survivor_map_visualizer.launch.py",
            condition=IfCondition(
                LaunchConfiguration("enable_raw_visualizer")
            ),
        ),
        _include("rescue_robot_survivor", "survivor_registry.launch.py"),
    ])
