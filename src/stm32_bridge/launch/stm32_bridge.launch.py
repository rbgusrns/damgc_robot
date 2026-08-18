from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttySTM0"),
        DeclareLaunchArgument("namespace", default_value="leader"),
        Node(
            package="stm32_bridge",
            executable="stm32_bridge_node",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "frame_id": "odom",
                "child_frame_id": "base_link",
                "imu_frame_id": "imu_link",
            }],
        ),
    ])
