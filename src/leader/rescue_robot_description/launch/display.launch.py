import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory(
        "rescue_robot_description"
    )

    urdf_path = os.path.join(
        package_path,
        "urdf",
        "rescue_robot.urdf"
    )

    rviz_path = os.path.join(
        package_path,
        "rviz",
        "urdf_model.rviz"
    )

    with open(urdf_path, "r", encoding="utf-8") as file:
        robot_description = file.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description
            }]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_path]
        )
    ])

