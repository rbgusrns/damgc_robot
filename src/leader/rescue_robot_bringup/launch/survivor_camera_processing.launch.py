"""Create rectified RGB for the survivor detector from the existing D435 topics.

This launch deliberately owns no camera driver, robot description, detector, or
navigation node.  The single RealSense instance is started by the existing
VSLAM mapping launcher.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch only CameraInfo bridging and RGB rectification."""
    return LaunchDescription(
        [
            Node(
                package="rescue_robot_apriltag",
                executable="camera_info_qos_bridge.py",
                name="survivor_camera_info_qos_bridge",
                parameters=[
                    {
                        "input_topic": "/leader/camera/color/camera_info",
                        "output_topic": (
                            "/leader/camera/color/camera_info_transient"
                        ),
                    }
                ],
                output="screen",
            ),
            Node(
                package="image_proc",
                executable="rectify_node",
                name="survivor_color_rectify",
                remappings=[
                    ("image", "/leader/camera/color/image_raw"),
                    (
                        "camera_info",
                        "/leader/camera/color/camera_info_transient",
                    ),
                    ("image_rect", "/leader/camera/color/image_rect"),
                ],
                parameters=[
                    {
                        "qos_overrides./camera_info.subscription.durability": (
                            "volatile"
                        )
                    }
                ],
                output="screen",
            ),
        ]
    )
