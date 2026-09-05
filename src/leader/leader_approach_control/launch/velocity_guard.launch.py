from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch only the Leader final software velocity guard."""
    default_config = PathJoinSubstitution(
        [FindPackageShare("leader_approach_control"), "config", "velocity_guard.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "guard_config",
                default_value=default_config,
                description="Leader velocity-guard parameter file",
            ),
            DeclareLaunchArgument(
                "guard_enabled_on_startup",
                default_value="false",
                choices=["true", "false"],
                description="Enable final guarded velocity output on startup",
            ),
            Node(
                package="leader_approach_control",
                executable="velocity_guard_node",
                namespace="leader",
                name="velocity_guard",
                parameters=[
                    LaunchConfiguration("guard_config"),
                    {
                        "guard_enabled_on_startup": ParameterValue(
                            LaunchConfiguration("guard_enabled_on_startup"),
                            value_type=bool,
                        )
                    },
                ],
                output="screen",
            ),
        ]
    )
