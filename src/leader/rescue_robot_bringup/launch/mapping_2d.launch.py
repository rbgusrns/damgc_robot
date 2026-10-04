"""Depth-only manual 2D SLAM and Nav2, with local wheel/IMU odometry."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('rescue_robot_bringup')
    description = get_package_share_directory('rescue_robot_description')
    with open(os.path.join(description, 'urdf', 'rescue_robot.urdf')) as source:
        urdf = source.read()
    return LaunchDescription([
        Node(package="rescue_robot_bringup", executable="imu_gyro_calibration.py",
             namespace="leader", output="screen"),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}], output='screen'),
        Node(package='robot_localization', executable='ekf_node',
             namespace='leader', name='ekf_localization_node',
             parameters=[os.path.join(share,'config','dual_ekf.yaml')],
             remappings=[('odometry/filtered','/leader/odometry/local')], output='screen'),
        Node(package='rescue_robot_bringup', executable='depth_obstacle_scan.py',
             output='screen'),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', parameters=[os.path.join(share,'config','slam_2d.yaml')],
             output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share,'launch','nvblox_nav2.launch.py')),
            launch_arguments={'params_file':os.path.join(share,'config','nav2_2d.yaml'),
                              'costmap_alias':'false'}.items()),
    ])
