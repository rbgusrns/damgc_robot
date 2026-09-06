from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("detection_topic", default_value="/leader/supply/detected"),
        DeclareLaunchArgument(
            "alignment_topic", default_value="/leader/base_alignment/state"
        ),
        DeclareLaunchArgument("raw_command_topic", default_value="/leader/dynamixel/command"),
        DeclareLaunchArgument("gripper_topic", default_value="/leader/gripper/command"),
        DeclareLaunchArgument("open_raw", default_value="1000"),
        DeclareLaunchArgument("close_raw", default_value="450"),
        DeclareLaunchArgument("close_wait", default_value="2.0"),
        DeclareLaunchArgument("lift_enabled", default_value="false"),
        DeclareLaunchArgument("lift_raw", default_value="-1"),
        Node(
            package="rescue_robot_tools",
            executable="gripper_sequence_node.py",
            name="gripper_sequence",
            output="screen",
            parameters=[{
                "enabled": LaunchConfiguration("enabled"),
                "detection_topic": LaunchConfiguration("detection_topic"),
                "alignment_topic": LaunchConfiguration("alignment_topic"),
                "raw_command_topic": LaunchConfiguration("raw_command_topic"),
                "gripper_topic": LaunchConfiguration("gripper_topic"),
                "open_raw": LaunchConfiguration("open_raw"),
                "close_raw": LaunchConfiguration("close_raw"),
                "close_wait": LaunchConfiguration("close_wait"),
                "lift_enabled": LaunchConfiguration("lift_enabled"),
                "lift_raw": ParameterValue(
                    LaunchConfiguration("lift_raw"), value_type=float
                ),
            }],
        ),
    ])
