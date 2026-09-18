"""Launch only the survivor map-position RViz visualizer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Expose marker topic and appearance parameters."""
    defaults = {
        "input_topic": "/leader/survivor/map_positions",
        "output_topic": "/leader/survivor/map_markers",
        "marker_namespace": "survivor_current",
        "marker_lifetime_sec": "2.0",
        "marker_scale": "0.20",
        "text_height": "0.18",
        "text_z_offset": "0.30",
    }
    actions = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]
    actions.append(Node(
        package="rescue_robot_survivor",
        executable="survivor_map_visualizer_node",
        namespace="leader",
        name="survivor_map_visualizer",
        parameters=[{
            name: (
                ParameterValue(LaunchConfiguration(name), value_type=float)
                if name in (
                    "marker_lifetime_sec", "marker_scale", "text_height",
                    "text_z_offset",
                ) else LaunchConfiguration(name)
            )
            for name in defaults
        }],
        output="screen",
    ))
    return LaunchDescription(actions)
