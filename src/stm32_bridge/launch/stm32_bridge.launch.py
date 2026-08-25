from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyTHS1"),
        DeclareLaunchArgument("baudrate", default_value="460800"),
        DeclareLaunchArgument("namespace", default_value="leader"),
        Node(
            package="stm32_bridge",
            executable="stm32_bridge_node",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "frame_id": "odom",
                "child_frame_id": "base_link",
                "imu_frame_id": "imu_link",
                "wheel_radius_m": 0.0635,
                "wheel_separation_m": 0.23,
                "ticks_per_revolution": 5131,
            }],
        ),
    ])
