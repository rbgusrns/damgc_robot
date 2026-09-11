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
    """Expose every Stage 1 detector parameter as a launch argument."""
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
                    }
                ],
                output="screen",
            ),
        ]
    )
