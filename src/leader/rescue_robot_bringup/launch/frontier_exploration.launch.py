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
        DeclareLaunchArgument("area_radius_m", default_value="0.0"),
        DeclareLaunchArgument("area_width_m", default_value="0.0"),
        DeclareLaunchArgument("area_height_m", default_value="0.0"),
        DeclareLaunchArgument("area_offset_x_m", default_value="0.0"),
        DeclareLaunchArgument("area_offset_y_m", default_value="0.0"),
        DeclareLaunchArgument("minimum_clearance_m", default_value="0.36"),
        DeclareLaunchArgument("connection_clearance_m", default_value="0.20"),
        DeclareLaunchArgument("minimum_goal_distance_m", default_value="0.60"),
        DeclareLaunchArgument("preferred_goal_distance_m", default_value="1.5"),
        DeclareLaunchArgument("minimum_cluster_cells", default_value="5"),
        DeclareLaunchArgument("minimum_unknown_area_m2", default_value="0.10"),
        DeclareLaunchArgument("goal_timeout_sec", default_value="90.0"),
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
                **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
                   for name in ("area_width_m", "area_height_m", "area_offset_x_m",
                                "area_offset_y_m", "minimum_clearance_m", "connection_clearance_m",
                                "minimum_goal_distance_m", "preferred_goal_distance_m", "minimum_unknown_area_m2", "goal_timeout_sec")},
                "minimum_cluster_cells": ParameterValue(
                    LaunchConfiguration("minimum_cluster_cells"), value_type=int),
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
