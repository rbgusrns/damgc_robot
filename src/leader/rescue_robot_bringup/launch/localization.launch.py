import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("rescue_robot_bringup")
    ekf_config = os.path.join(package_share, "config", "dual_ekf.yaml")

    local_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_localization_node",
        namespace="leader",
        parameters=[ekf_config],
        remappings=[("odometry/filtered", "/leader/odometry/local")],
        output="screen",
    )

    global_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_globalization_node",
        namespace="leader",
        parameters=[ekf_config],
        remappings=[("odometry/filtered", "/leader/odometry/global")],
        output="screen",
    )

    vslam_covariance_adapter = Node(
        package="rescue_robot_bringup",
        executable="vslam_covariance_adapter.py",
        name="vslam_covariance_adapter",
        namespace="leader",
        output="screen",
    )

    return LaunchDescription([
        local_ekf,
        vslam_covariance_adapter,
        global_ekf,
    ])
