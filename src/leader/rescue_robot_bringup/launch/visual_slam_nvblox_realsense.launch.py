import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(name):
    path = os.path.join(
        get_package_share_directory("rescue_robot_bringup"),
        "launch",
        name,
    )
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))


def generate_launch_description():
    actions = [
        _include("localization.launch.py"),
        _include("visual_slam_realsense.launch.py"),
    ]
    if os.environ.get("DAMGC_VSLAM_ONLY", "0") != "1":
        actions.append(_include("nvblox_realsense.launch.py"))
    return LaunchDescription(actions)
