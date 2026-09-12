"""Launch only the Leader survivor person detector."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _configuration_defaults():
    config_path = os.path.join(
        get_package_share_directory("rescue_robot_survivor"),
        "config",
        "person_detector.yaml",
    )
    with open(config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    return config["/leader/person_detector"]["ros__parameters"]


def generate_launch_description() -> LaunchDescription:
    """Expose every survivor detector parameter as a launch argument."""
    defaults = _configuration_defaults()
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic", default_value=str(defaults["image_topic"])
            ),
            DeclareLaunchArgument(
                "debug_image_topic",
                default_value=str(defaults["debug_image_topic"]),
            ),
            DeclareLaunchArgument(
                "model_name", default_value=str(defaults["model_name"])
            ),
            DeclareLaunchArgument(
                "confidence_threshold",
                default_value=str(defaults["confidence_threshold"]),
            ),
            DeclareLaunchArgument(
                "device", default_value=str(defaults["device"])
            ),
            DeclareLaunchArgument(
                "aligned_depth_topic",
                default_value=str(defaults["aligned_depth_topic"]),
            ),
            DeclareLaunchArgument(
                "depth_roi_width_ratio",
                default_value=str(defaults["depth_roi_width_ratio"]),
            ),
            DeclareLaunchArgument(
                "depth_roi_height_ratio",
                default_value=str(defaults["depth_roi_height_ratio"]),
            ),
            DeclareLaunchArgument(
                "min_depth_m", default_value=str(defaults["min_depth_m"])
            ),
            DeclareLaunchArgument(
                "max_depth_m", default_value=str(defaults["max_depth_m"])
            ),
            DeclareLaunchArgument(
                "min_valid_depth_pixels",
                default_value=str(defaults["min_valid_depth_pixels"]),
            ),
            DeclareLaunchArgument(
                "depth_scale_m_per_unit",
                default_value=str(defaults["depth_scale_m_per_unit"]),
            ),
            DeclareLaunchArgument(
                "show_depth_roi",
                default_value=str(defaults["show_depth_roi"]).lower(),
            ),
            DeclareLaunchArgument(
                "sync_queue_size",
                default_value=str(defaults["sync_queue_size"]),
            ),
            DeclareLaunchArgument(
                "sync_slop_sec", default_value=str(defaults["sync_slop_sec"])
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value=str(defaults["camera_info_topic"]),
            ),
            DeclareLaunchArgument(
                "camera_positions_topic",
                default_value=str(defaults["camera_positions_topic"]),
            ),
            DeclareLaunchArgument(
                "show_camera_xyz",
                default_value=str(defaults["show_camera_xyz"]).lower(),
            ),
            Node(
                package="rescue_robot_survivor",
                executable="person_detector_node",
                namespace="leader",
                name="person_detector",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "debug_image_topic": LaunchConfiguration(
                            "debug_image_topic"
                        ),
                        "model_name": LaunchConfiguration("model_name"),
                        "confidence_threshold": ParameterValue(
                            LaunchConfiguration("confidence_threshold"),
                            value_type=float,
                        ),
                        "device": LaunchConfiguration("device"),
                        "aligned_depth_topic": LaunchConfiguration(
                            "aligned_depth_topic"
                        ),
                        "depth_roi_width_ratio": ParameterValue(
                            LaunchConfiguration("depth_roi_width_ratio"),
                            value_type=float,
                        ),
                        "depth_roi_height_ratio": ParameterValue(
                            LaunchConfiguration("depth_roi_height_ratio"),
                            value_type=float,
                        ),
                        "min_depth_m": ParameterValue(
                            LaunchConfiguration("min_depth_m"), value_type=float
                        ),
                        "max_depth_m": ParameterValue(
                            LaunchConfiguration("max_depth_m"), value_type=float
                        ),
                        "min_valid_depth_pixels": ParameterValue(
                            LaunchConfiguration("min_valid_depth_pixels"),
                            value_type=int,
                        ),
                        "depth_scale_m_per_unit": ParameterValue(
                            LaunchConfiguration("depth_scale_m_per_unit"),
                            value_type=float,
                        ),
                        "show_depth_roi": ParameterValue(
                            LaunchConfiguration("show_depth_roi"),
                            value_type=bool,
                        ),
                        "sync_queue_size": ParameterValue(
                            LaunchConfiguration("sync_queue_size"),
                            value_type=int,
                        ),
                        "sync_slop_sec": ParameterValue(
                            LaunchConfiguration("sync_slop_sec"),
                            value_type=float,
                        ),
                        "camera_info_topic": LaunchConfiguration(
                            "camera_info_topic"
                        ),
                        "camera_positions_topic": LaunchConfiguration(
                            "camera_positions_topic"
                        ),
                        "show_camera_xyz": ParameterValue(
                            LaunchConfiguration("show_camera_xyz"),
                            value_type=bool,
                        ),
                    }
                ],
                output="screen",
            ),
        ]
    )
