from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="leader_cooperation",
            executable="leader_cooperation_node",
            name="leader_cooperation",
            output="screen",
        )
    ])
