"""Launch Stage 3 temporal processing; Stage 2 and camera run separately."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        ("raw_topic", "/leader/survivor/eye_state/raw"),
        ("stage2_debug_image_topic", "/leader/survivor/eye_state/debug_image"),
        ("output_topic", "/leader/survivor/eye_state/stable"),
        ("debug_image_topic", "/leader/survivor/eye_state/temporal_debug_image"),
        ("history_window_sec", "3.0"),
        ("min_valid_samples", "5"),
        ("min_valid_coverage", "0.5"),
        ("open_ratio_threshold", "0.70"),
        ("closed_ratio_threshold", "0.70"),
        ("track_timeout_sec", "1.5"),
        ("match_iou_threshold", "0.25"),
        ("match_center_distance_threshold", "0.30"),
    ]
    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in arguments
    ]
    float_parameters = {
        "history_window_sec", "min_valid_coverage", "open_ratio_threshold",
        "closed_ratio_threshold", "track_timeout_sec", "match_iou_threshold",
        "match_center_distance_threshold",
    }
    int_parameters = {"min_valid_samples"}
    parameters = {}
    for name, _ in arguments:
        value = LaunchConfiguration(name)
        if name in float_parameters:
            parameters[name] = ParameterValue(value, value_type=float)
        elif name in int_parameters:
            parameters[name] = ParameterValue(value, value_type=int)
        else:
            parameters[name] = value

    temporal_node = Node(
        package="rescue_robot_eye_state",
        executable="survivor_eye_state_temporal_node.py",
        name="survivor_eye_state_temporal_node",
        output="screen",
        parameters=[parameters],
    )
    return LaunchDescription([*declarations, temporal_node])
