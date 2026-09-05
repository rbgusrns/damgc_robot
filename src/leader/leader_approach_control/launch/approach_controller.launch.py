from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch only the software-level Leader raw approach controller."""
    default_config = PathJoinSubstitution(
        [FindPackageShare("leader_approach_control"), "config", "approach_controller.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller_config",
                default_value=default_config,
                description="Leader approach-controller parameter file",
            ),
            DeclareLaunchArgument(
                "controller_enabled_on_startup",
                default_value="false",
                choices=["true", "false"],
                description="Enable raw approach command generation on startup",
            ),
            Node(
                package="leader_approach_control",
                executable="approach_controller_node",
                namespace="leader",
                name="approach_controller",
                parameters=[
                    LaunchConfiguration("controller_config"),
                    {
                        "controller_enabled_on_startup": ParameterValue(
                            LaunchConfiguration("controller_enabled_on_startup"),
                            value_type=bool,
                        )
                    },
                ],
                output="screen",
            ),
        ]
    )
