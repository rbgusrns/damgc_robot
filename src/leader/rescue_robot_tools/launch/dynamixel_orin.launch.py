from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("robot", default_value="leader"),
        DeclareLaunchArgument("startup_rx64_raw", default_value="600"),
        DeclareLaunchArgument("startup_rx28_raw", default_value="500"),
        DeclareLaunchArgument("startup_torque", default_value="true"),
        Node(
            package="rescue_robot_tools",
            executable="dynamixel_orin_node.py",
            name="dynamixel_orin_node",
            namespace=LaunchConfiguration("robot"),
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "robot": LaunchConfiguration("robot"),
                "startup_rx64_raw": LaunchConfiguration("startup_rx64_raw"),
                "startup_rx28_raw": LaunchConfiguration("startup_rx28_raw"),
                "startup_torque": LaunchConfiguration("startup_torque"),
            }],
        ),
    ])
