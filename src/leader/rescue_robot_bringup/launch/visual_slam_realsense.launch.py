from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    headless = os.environ.get("DAMGC_VSLAM_HEADLESS", "0") == "1"
    description_share = get_package_share_directory("rescue_robot_description")
    robot_description_path = os.path.join(
        description_share, "urdf", "rescue_robot.urdf")
    with open(robot_description_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_vslam",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    visual_slam = Node(
        package="isaac_ros_visual_slam",
        executable="isaac_ros_visual_slam",
        name="visual_slam_node",
        parameters=[{
            "enable_image_denoising": False,
            "rectified_images": True,
            # The connected device is D435 (not D435i), so use stereo-only VO.
            "enable_imu_fusion": False,
            "gyro_noise_density": 0.000244,
            "gyro_random_walk": 0.000019393,
            "accel_noise_density": 0.001862,
            "accel_random_walk": 0.003,
            "calibration_frequency": 200.0,
            "image_jitter_threshold_ms": 22.0,
            # Use the RealSense camera frame for the first tracking test.
            # A later integration step will connect this frame to base_link.
            "base_frame": "base_link",
            # robot_localization owns both TF links in the fused setup.
            "publish_odom_to_base_tf": False,
            "publish_map_to_odom_tf": False,
            # Debug renderings are optional; odometry/status and rosbag output
            # remain active in headless mode.
            "enable_slam_visualization": not headless,
            "enable_landmarks_view": not headless,
            "enable_observations_view": not headless,
            "camera_optical_frames": [
                "camera_infra1_optical_frame",
                "camera_infra2_optical_frame",
            ],
        }],
        remappings=[
            ("visual_slam/image_0", "/leader/camera/infra1/image_rect_raw"),
            ("visual_slam/camera_info_0", "/leader/camera/infra1/camera_info"),
            ("visual_slam/image_1", "/leader/camera/infra2/image_rect_raw"),
            ("visual_slam/camera_info_1", "/leader/camera/infra2/camera_info"),
        ],
    )

    return LaunchDescription([robot_state_publisher, visual_slam])
