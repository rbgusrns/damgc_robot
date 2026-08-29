"""Launch only the Follower raw AprilTag approach controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Load the conservative controller configuration under /follower."""
    default_config = PathJoinSubstitution(
        [FindPackageShare("follower_approach_control"), "config", "approach_controller.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller_config",
                default_value=default_config,
                description="Follower approach-controller parameter file",
            ),
            Node(
                package="follower_approach_control",
                executable="approach_controller_node",
                namespace="/follower",
                name="approach_controller",
                parameters=[LaunchConfiguration("controller_config")],
                output="screen",
            ),
        ]
    )
