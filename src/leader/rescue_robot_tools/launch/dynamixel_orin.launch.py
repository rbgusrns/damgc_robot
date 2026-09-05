from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("robot", default_value="leader"),
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
            }],
        ),
    ])
