"""Arrival-only detector sharing mapping's existing RGB/depth camera."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include(package, filename, **arguments):
    return GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', filename)),
        launch_arguments={k: str(v) for k, v in arguments.items()}.items())],
        scoped=True, forwarding=False)


def generate_launch_description():
    root = os.environ['DAMGC_REPO_ROOT']
    cache = os.environ.get('SURVIVOR_MODEL_CACHE', os.path.expanduser('~/.cache/damgc-survivor-ultralytics'))
    os.makedirs(cache, exist_ok=True)
    command = ['docker', 'run', '--rm', '--name', os.environ['CHECKPOINT_CONTAINER_NAME'],
               '--runtime=nvidia', '--network', 'host', '--ipc', 'host',
               '-e', 'NVIDIA_VISIBLE_DEVICES=all',
               '-e', 'NVIDIA_DRIVER_CAPABILITIES=compute,utility',
               '-e', 'YOLO_CONFIG_DIR=/tmp']
    for key, default in [('ROS_DOMAIN_ID', '0'), ('ROS_LOCALHOST_ONLY', '0'),
                         ('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
                         ('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')]:
        command += ['-e', f'{key}={os.environ.get(key, default)}']
    command += ['-v', f'{root}:/workspaces/isaac_ros-dev:ro',
                '-v', f'{cache}:/root/.cache/ultralytics',
                '--workdir', '/root/.cache/ultralytics',
                os.environ.get('SURVIVOR_IMAGE', 'damgc-survivor-yolo:humble'),
                'source /opt/ros/humble/setup.bash; '
                'source /opt/damgc_survivor_ws/install_survivor/setup.bash; '
                'export PYTHONPATH=/workspaces/isaac_ros-dev/src/leader/rescue_robot_survivor:$PYTHONPATH; '
                'exec python3 -m rescue_robot_survivor.person_detector_node --ros-args '
                '-r __ns:=/leader '
                '--params-file /workspaces/isaac_ros-dev/src/leader/rescue_robot_survivor/config/person_detector.yaml '
                '-p checkpoint_mode:=true -p inference_hz:=2.0']
    return LaunchDescription([
        include('rescue_robot_bringup', 'survivor_camera_processing.launch.py'),
        ExecuteProcess(cmd=command, output='screen'),
        include('rescue_robot_survivor', 'survivor_map_transform.launch.py', target_frame='odom'),
        include('rescue_robot_survivor', 'survivor_registry.launch.py', map_frame='odom',
                confirm_min_hits=3, confirm_min_duration_sec=0.0,
                tentative_max_gap_sec=12.0),
    ])
