"""Use wheel odometry for local mapping and Nav2; observe VSLAM independently.

Do not run another odom -> base_link publisher (including dual EKF) alongside.
VSLAM does not correct this local frame until visual translation is validated.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(name, arguments=None):
    path = os.path.join(os.path.dirname(__file__), name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    gxf_library_paths = [
        "/opt/ros/humble/lib",
        "/opt/ros/humble/share/isaac_ros_gxf/gxf/lib",
        "/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/serialization",
        "/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/logger",
    ]
    existing_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    if existing_library_path:
        gxf_library_paths.append(existing_library_path)

    return LaunchDescription([
        DeclareLaunchArgument("filter_enabled", default_value="true"),
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
        SetEnvironmentVariable("LD_LIBRARY_PATH", os.pathsep.join(gxf_library_paths)),
        _include("visual_slam_realsense.launch.py", {
            "publish_odom_to_base_tf": "false",
            "publish_map_to_odom_tf": "false",
            "image_jitter_threshold_ms": "50.0",
        }),
        Node(package="rescue_robot_bringup", executable="wheel_odometry_tf.py",
             name="wheel_odometry_tf", output="screen"),
        _include("nvblox_realsense.launch.py", {
            "filter_enabled": LaunchConfiguration("filter_enabled"),
        }),
        _include("nvblox_nav2.launch.py"),
    ])
