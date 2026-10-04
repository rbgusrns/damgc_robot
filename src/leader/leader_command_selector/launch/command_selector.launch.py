"""Launch the deterministic Leader velocity command selector."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch one selector under the Leader namespace."""
    default_config = PathJoinSubstitution(
        [FindPackageShare("leader_command_selector"), "config", "command_selector.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "selector_config",
                default_value=default_config,
                description="Leader command-selector parameter file",
            ),
            DeclareLaunchArgument(
                "source_mode",
                default_value="STOP",
                choices=["STOP", "TELEOP", "APPROACH", "NAV2", "COOPERATION"],
                description="Initial Leader velocity command source",
            ),
            DeclareLaunchArgument("enable_nav2_goal_selection", default_value="false"),
            Node(
                package="leader_command_selector",
                executable="command_selector_node",
                namespace="leader",
                name="command_selector",
                parameters=[
                    LaunchConfiguration("selector_config"),
                    {
                        "enable_nav2_goal_selection": ParameterValue(
                            LaunchConfiguration("enable_nav2_goal_selection"), value_type=bool
                        ),
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
