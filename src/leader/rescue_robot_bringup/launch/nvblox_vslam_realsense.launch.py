"""Map with wheel/IMU odometry for Nav2 and VSLAM for visual mapping."""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        DeclareLaunchArgument("initial_scan", default_value="false"),
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
        SetEnvironmentVariable("LD_LIBRARY_PATH", os.pathsep.join(gxf_library_paths)),
        _include("localization.launch.py"),
        _include("visual_slam_realsense.launch.py", {
            "publish_odom_to_base_tf": "false",
            "publish_map_to_odom_tf": "false",
            "image_jitter_threshold_ms": "50.0",
        }),
        _include("nvblox_realsense.launch.py", {
            "filter_enabled": LaunchConfiguration("filter_enabled"),
        }),
        _include("nvblox_nav2.launch.py"),
        Node(package="cooperative_transport", executable="transport_peer",
             namespace="leader", output="screen", parameters=[{"role": "leader"}]),
        Node(package="rescue_robot_bringup", executable="mapping_mode_manager.py",
             namespace="leader", output="screen", parameters=[os.path.join(
                 get_package_share_directory("rescue_robot_bringup"), "config", "carrying_mode.yaml")]),
        Node(package="rescue_robot_bringup", executable="nvblox_slice_map.py",
             name="nvblox_slice_map", output="screen"),
        Node(
            package="rescue_robot_bringup",
            executable="initial_map_scan.py",
            name="initial_map_scan",
            output="screen",
            parameters=[{
                "enabled": ParameterValue(
                    LaunchConfiguration("initial_scan"), value_type=bool
                ),
                "odometry_topic": "/leader/odometry/local",
                "selector_node": "/leader/command_selector",
                "spin_action": "/spin",
                "target_yaw": 6.2831853,
                "startup_timeout": 300.0,
            }],
        ),
    ])
