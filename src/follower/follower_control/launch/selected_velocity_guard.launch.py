"""Connect the command selector output to the final Follower safety guard."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    """Launch the guard in selector-integrated software-control mode."""
    config = PathJoinSubstitution(
        [FindPackageShare("follower_control"), "config", "velocity_guard.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="follower_control",
                executable="velocity_guard_node",
                namespace="/follower",
                name="velocity_guard",
                parameters=[
                    config,
                    {"command_topic": "/follower/selected_cmd_vel"},
                ],
                output="screen",
            )
        ]
    )
