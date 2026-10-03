"""Launch the standalone Eye State node without the Survivor pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "image_topic",
            default_value="/leader/camera/color/image_raw",
            description="RGB sensor_msgs/Image topic",
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value=(
                "/home/maze/eye_state_runs/eye_state/baseline/weights/"
                "eye_state_best.pt"
            ),
            description="YOLO11n-cls checkpoint path",
        ),
        DeclareLaunchArgument(
            "yunet_model_path",
            default_value=(
                "/home/maze/.cache/damgc-eye-state/"
                "face_detection_yunet_2022mar.onnx"
            ),
            description="Pinned OpenCV Zoo YuNet ONNX model path",
        ),
        DeclareLaunchArgument("face_conf_threshold", default_value="0.60"),
        DeclareLaunchArgument("eye_conf_threshold", default_value="0.60"),
        DeclareLaunchArgument("min_eye_width", default_value="24"),
        DeclareLaunchArgument("min_eye_height", default_value="24"),
        DeclareLaunchArgument("inference_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("preprocess_mode", default_value="gray"),
        DeclareLaunchArgument("show_landmarks", default_value="true"),
        DeclareLaunchArgument(
            "device",
            default_value="cpu",
            description="Classifier device: cpu or CUDA device index such as 0",
        ),
        Node(
            package="rescue_robot_eye_state",
            executable="survivor_eye_state_node.py",
            name="survivor_eye_state_node",
            output="screen",
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "model_path": LaunchConfiguration("model_path"),
                "yunet_model_path": LaunchConfiguration("yunet_model_path"),
                "face_conf_threshold": ParameterValue(
                    LaunchConfiguration("face_conf_threshold"), value_type=float
                ),
                "eye_conf_threshold": ParameterValue(
                    LaunchConfiguration("eye_conf_threshold"), value_type=float
                ),
                "min_eye_width": ParameterValue(
                    LaunchConfiguration("min_eye_width"), value_type=int
                ),
                "min_eye_height": ParameterValue(
                    LaunchConfiguration("min_eye_height"), value_type=int
                ),
                "inference_rate_hz": ParameterValue(
                    LaunchConfiguration("inference_rate_hz"), value_type=float
                ),
                "preprocess_mode": LaunchConfiguration("preprocess_mode"),
                "show_landmarks": ParameterValue(
                    LaunchConfiguration("show_landmarks"), value_type=bool
                ),
                "device": ParameterValue(
                    LaunchConfiguration("device"), value_type=str
                ),
            }],
        ),
    ])
