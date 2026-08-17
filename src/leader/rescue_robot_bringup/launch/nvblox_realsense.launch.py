from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nvblox = Node(
        package="nvblox_ros",
        executable="nvblox_node",
        name="nvblox_node",
        output="screen",
        parameters=[{
            "num_cameras": 1,
            "use_tf_transforms": True,
            "mapping_type": "static_tsdf",
            "voxel_size": 0.05,
            "use_depth": True,
            "use_color": True,
            "use_lidar": False,
            "esdf_mode": "3d",
            "map_clearing_frame_id": "base_link",
            "esdf_slice_bounds_visualization_attachment_frame_id": "base_link",
            "workspace_height_bounds_type": "unbounded",
            "input_qos": "SENSOR_DATA",
            "integrate_depth_rate_hz": 30.0,
            "integrate_color_rate_hz": 5.0,
            "update_mesh_rate_hz": 5.0,
            "publish_layer_rate_hz": 5.0,
        }],
        remappings=[
            ("camera_0/depth/image", "/leader/camera/depth/image_rect_raw"),
            ("camera_0/depth/camera_info", "/leader/camera/depth/camera_info"),
            ("camera_0/color/image", "/leader/camera/color/image_raw"),
            ("camera_0/color/camera_info", "/leader/camera/color/camera_info"),
        ],
    )

    return LaunchDescription([nvblox])
