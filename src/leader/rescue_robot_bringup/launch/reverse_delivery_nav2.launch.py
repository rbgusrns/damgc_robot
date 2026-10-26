"""Reverse delivery phase only: existing odometry, frozen map, planner and controller."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config=os.path.join(get_package_share_directory('rescue_robot_bringup'),'config','nvblox_nav2.yaml')
    remap=[('/nvblox_node/static_map_slice','/mapping/navigation_slice')]
    return LaunchDescription([
        Node(package='nav2_planner',executable='planner_server',name='planner_server',
             parameters=[config],remappings=remap,output='screen'),
        Node(package='nav2_controller',executable='controller_server',name='controller_server',
             parameters=[config],remappings=remap+[('cmd_vel','/nav2/cmd_vel')],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='delivery_lifecycle_manager',
             parameters=[{'autostart':True,'bond_timeout':0.,'node_names':['planner_server','controller_server']}],output='screen'),
    ])
