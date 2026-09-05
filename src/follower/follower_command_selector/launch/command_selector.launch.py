"""Launch the deterministic Follower velocity command selector."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch one selector under the Follower namespace."""
    default_config = PathJoinSubstitution(
        [FindPackageShare("follower_command_selector"), "config", "command_selector.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "selector_config",
                default_value=default_config,
                description="Follower command-selector parameter file",
            ),
            DeclareLaunchArgument(
                "source_mode",
                default_value="STOP",
                choices=["STOP", "APPROACH", "COOPERATION"],
                description="Initial Follower velocity command source",
            ),
            Node(
                package="follower_command_selector",
                executable="command_selector_node",
                namespace="/follower",
                name="command_selector",
                parameters=[
                    LaunchConfiguration("selector_config"),
                    {
                        "source_mode": ParameterValue(
                            LaunchConfiguration("source_mode"),
                            value_type=str,
                        )
                    },
                ],
                output="screen",
            ),
        ]
    )
