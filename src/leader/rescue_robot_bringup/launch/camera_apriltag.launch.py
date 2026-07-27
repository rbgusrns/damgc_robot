import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depth_enabled = LaunchConfiguration("enable_depth")
    description_share = get_package_share_directory("rescue_robot_description")
    apriltag_share = get_package_share_directory("rescue_robot_apriltag")
    robot_description_path = os.path.join(description_share, "urdf", "rescue_robot.urdf")
    with open(robot_description_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    realsense_launch = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable_depth", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                "camera_namespace": "leader",
                "camera_name": "camera",
                "enable_color": "true",
                "enable_depth": depth_enabled,
                "enable_infra": "false",
                "enable_infra1": "false",
                "enable_infra2": "false",
                "rgb_camera.color_profile": "640x480x30",
                "depth_module.depth_profile": "640x480x30",
            }.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="rescue_robot_apriltag",
            executable="camera_info_qos_bridge.py",
            name="camera_info_qos_bridge",
            output="screen",
        ),
        Node(
            package="image_proc",
            executable="rectify_node",
            name="RectifyNode",
            remappings=[
                ("image", "/leader/camera/color/image_raw"),
                ("camera_info", "/leader/camera/color/camera_info_transient"),
                ("image_rect", "/leader/camera/color/image_rect"),
            ],
            parameters=[{"qos_overrides./camera_info.subscription.durability": "volatile"}],
            output="screen",
        ),
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            namespace="leader/apriltag",
            name="apriltag",
            parameters=[os.path.join(apriltag_share, "config", "apriltag_leader.yaml")],
            remappings=[
                ("image_rect", "/leader/camera/color/image_rect"),
                ("camera_info", "/leader/camera/color/camera_info"),
            ],
            output="screen",
        ),
    ])
