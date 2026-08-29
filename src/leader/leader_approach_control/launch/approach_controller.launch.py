from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
            Node(
                package="leader_approach_control",
                executable="approach_controller_node",
                namespace="leader",
                name="approach_controller",
                parameters=[LaunchConfiguration("controller_config")],
                output="screen",
            ),
        ]
    )
