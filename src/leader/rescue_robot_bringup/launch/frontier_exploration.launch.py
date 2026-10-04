"""Launch the bounded frontier exploration coordinator alongside Nav2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("area_radius_m", default_value="2.0"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        Node(
            package="rescue_robot_bringup",
            executable="frontier_explorer.py",
            name="frontier_explorer",
            output="screen",
            parameters=[{
                "area_radius_m": ParameterValue(
                    LaunchConfiguration("area_radius_m"), value_type=float
                ),
                "dry_run": ParameterValue(
                    LaunchConfiguration("dry_run"), value_type=bool
                ),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="frontier_rviz",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("rescue_robot_bringup"),
                "rviz",
                "frontier_exploration.rviz",
            ])],
            condition=IfCondition(LaunchConfiguration("start_rviz")),
            output="screen",
        ),
    ])
