"""Launch only the follower AprilTag approach-state node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create an approach-only launch description for an existing tag pipeline."""
    approach_config = LaunchConfiguration("approach_config")
    default_config = PathJoinSubstitution(
        [FindPackageShare("follower_supply_perception"), "config", "approach.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "approach_config",
                default_value=default_config,
                description="Absolute path to the approach-state parameter YAML",
            ),
            Node(
                package="follower_supply_perception",
                executable="apriltag_approach_node",
                namespace="/follower",
                name="apriltag_approach",
                output="screen",
                parameters=[approach_config],
            ),
        ]
    )
