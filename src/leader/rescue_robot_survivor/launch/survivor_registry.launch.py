"""Launch the persistent Registry and its independent RViz visualizer."""

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
        "survivor_registry.yaml",
    )
    with open(config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    return (
        config["/leader/survivor_registry"]["ros__parameters"],
        config["/leader/survivor_registry_visualizer"]["ros__parameters"],
    )


def generate_launch_description():
    """Expose Stage 6 parameters without starting upstream nodes."""
    defaults, visualizer_defaults = _configuration_defaults()
    float_parameters = {
        "association_radius_m",
        "reassociation_radius_m",
        "confirm_min_duration_sec",
        "tentative_max_gap_sec",
        "visible_timeout_sec",
        "position_ema_alpha",
        "registry_publish_hz",
    }
    actions = [
        DeclareLaunchArgument(name, default_value=str(value))
        for name, value in defaults.items()
    ]
    actions.extend([
        DeclareLaunchArgument(
            "registry_markers_topic",
            default_value=str(visualizer_defaults["output_topic"]),
        ),
        *(
            DeclareLaunchArgument(name, default_value=str(
                visualizer_defaults[name]
            ))
            for name in (
                "marker_namespace",
                "marker_scale",
                "text_height",
                "text_z_offset",
            )
        ),
    ])
    actions.append(Node(
        package="rescue_robot_survivor",
        executable="survivor_registry_node",
        namespace="leader",
        name="survivor_registry",
        parameters=[{
            name: (
                ParameterValue(LaunchConfiguration(name), value_type=float)
                if name in float_parameters else
                ParameterValue(LaunchConfiguration(name), value_type=int)
                if name == "confirm_min_hits" else
                LaunchConfiguration(name)
            )
            for name in defaults
        }],
        output="screen",
    ))
    actions.append(Node(
        package="rescue_robot_survivor",
        executable="survivor_registry_visualizer_node",
        namespace="leader",
        name="survivor_registry_visualizer",
        parameters=[{
            "input_topic": LaunchConfiguration("tracks_topic"),
            "output_topic": LaunchConfiguration(
                "registry_markers_topic"
            ),
            "map_frame": LaunchConfiguration("map_frame"),
            "marker_namespace": LaunchConfiguration("marker_namespace"),
            "marker_scale": ParameterValue(
                LaunchConfiguration("marker_scale"), value_type=float
            ),
            "text_height": ParameterValue(
                LaunchConfiguration("text_height"), value_type=float
            ),
            "text_z_offset": ParameterValue(
                LaunchConfiguration("text_z_offset"), value_type=float
            ),
        }],
        output="screen",
    ))
    return LaunchDescription(actions)
