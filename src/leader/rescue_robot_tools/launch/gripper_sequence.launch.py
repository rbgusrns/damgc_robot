from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument(
            "alignment_topic",
            default_value="/leader/apriltag_approach/alignment/state",
        ),
        DeclareLaunchArgument(
            "gripper_topic", default_value="/leader/gripper/command"
        ),
        Node(
            package="rescue_robot_tools",
            executable="gripper_sequence_node.py",
            name="gripper_sequence",
            output="screen",
            parameters=[{
                "enabled": LaunchConfiguration("enabled"),
                "alignment_topic": LaunchConfiguration("alignment_topic"),
                "gripper_topic": LaunchConfiguration("gripper_topic"),
            }],
        ),
    ])
