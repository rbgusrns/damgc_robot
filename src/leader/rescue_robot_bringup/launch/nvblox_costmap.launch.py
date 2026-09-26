"""Run a Nav2 local costmap from the nvblox static distance-map slice."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("rescue_robot_bringup"),
        "config",
        "nvblox_costmap.yaml",
    )
    costmap = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        output="screen",
        parameters=[params_file],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_nvblox_costmap",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "bond_timeout": 0.0,
            "node_names": ["/costmap/costmap"],
        }],
    )
    return LaunchDescription([costmap, lifecycle_manager])
