from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("transport", default_value="i2c"),
        DeclareLaunchArgument("i2c_device", default_value="/dev/i2c-7"),
        DeclareLaunchArgument("i2c_address", default_value="66"),
        DeclareLaunchArgument("i2c_read_size", default_value="64"),
        DeclareLaunchArgument("i2c_poll_hz", default_value="500.0"),
        DeclareLaunchArgument("i2c_write_enabled", default_value="true"),
        DeclareLaunchArgument("port", default_value="/dev/ttyTHS1"),
        DeclareLaunchArgument("baudrate", default_value="230400"),
        DeclareLaunchArgument("namespace", default_value="leader"),
        Node(
            package="stm32_bridge",
            executable="stm32_bridge_node",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=[{
                "transport": LaunchConfiguration("transport"),
                "i2c_device": LaunchConfiguration("i2c_device"),
                "i2c_address": LaunchConfiguration("i2c_address"),
                "i2c_read_size": LaunchConfiguration("i2c_read_size"),
                "i2c_poll_hz": LaunchConfiguration("i2c_poll_hz"),
                "i2c_write_enabled": LaunchConfiguration("i2c_write_enabled"),
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
