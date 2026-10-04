#!/usr/bin/env python3
"""Save nvblox layers and a stationary odom-frame restart pose, or load layers."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from nvblox_msgs.srv import FilePath

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('operation', choices=['save', 'load'])
parser.add_argument('directory')
args = parser.parse_args()
directory = Path(args.directory).resolve()
rclpy.init()
node = Node('mapping_snapshot')
pose = None
grid = None
stationary_since = None
last_odom_received = None

def odom(message):
    global pose, stationary_since, last_odom_received
    pose = message
    last_odom_received = time.monotonic()
    moving = abs(message.twist.twist.linear.x) > .005 or abs(message.twist.twist.angular.z) > .01
    stationary_since = None if moving else (stationary_since or time.monotonic())

def map_callback(message):
    global grid
    grid = message

node.create_subscription(Odometry, '/leader/odometry/local', odom, qos_profile_sensor_data)
node.create_subscription(OccupancyGrid, '/mapping/projected_map', map_callback,
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
try:
    if args.operation == 'save':
        directory.mkdir(parents=True, exist_ok=False)
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if pose and stationary_since and time.monotonic()-stationary_since >= 2 and time.monotonic()-last_odom_received < .5:
                break
        if not pose or not stationary_since or time.monotonic()-stationary_since < 2 or time.monotonic()-last_odom_received >= .5:
            raise RuntimeError('Robot must be stationary with fresh local odometry')
        if pose.header.frame_id != 'odom':
            raise RuntimeError('Snapshot requires odom-frame pose')
        p = pose.pose.pose
        q = p.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        manifest = {'format_version': 1, 'map_frame': 'odom', 'mapping_session_id': Path('/workspaces/isaac_ros-dev/data/.mapping_session_id').read_text().strip() if Path('/workspaces/isaac_ros-dev/data/.mapping_session_id').exists() else None, 'base_frame': pose.child_frame_id,
                    'nvblox_file': 'map.nvblx', 'saved_at_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                    'anchor': {'x': p.position.x, 'y': p.position.y, 'yaw_rad': yaw,
                               'yaw_deg': math.degrees(yaw)},
                    'initial_state': [p.position.x,p.position.y,0.,0.,0.,yaw]+[0.]*9,
                    'wheel_separation_m': .2453246, 'gyro_z_scale': .9684782203835076,
                    'requires_same_physical_position_and_heading': True}
    else:
        manifest = json.loads((directory/'manifest.json').read_text())
        if manifest['map_frame'] != 'odom':
            raise RuntimeError('Unsupported map frame')
    map_path = directory/manifest['nvblox_file']
    if args.operation == 'load' and (not map_path.is_file() or map_path.stat().st_size == 0):
        raise RuntimeError('Missing map layers')
    client = node.create_client(FilePath, '/nvblox_node/'+args.operation+'_map')
    if not client.wait_for_service(timeout_sec=30):
        raise RuntimeError('nvblox service unavailable')
    request = FilePath.Request()
    request.file_path = str(map_path)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=60)
    if not future.done() or not future.result() or not future.result().success:
        raise RuntimeError('nvblox '+args.operation+' failed')
    if args.operation == 'save':
        if not map_path.is_file() or map_path.stat().st_size == 0:
            raise RuntimeError('Saved map is empty')
        manifest['map_bytes'] = map_path.stat().st_size
        if grid is not None:
            # Export the display projection for inspection. nvblox layers are authoritative.
            values = grid.data
            pixels = bytearray()
            for row in range(grid.info.height-1, -1, -1):
                for col in range(grid.info.width):
                    v = values[row*grid.info.width+col]
                    pixels.append(205 if v < 0 else 0 if v >= 65 else 254)
            (directory/'projection.pgm').write_bytes(
                f'P5\n{grid.info.width} {grid.info.height}\n255\n'.encode()+pixels)
            origin = grid.info.origin
            q = origin.orientation
            grid_yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            (directory/'projection.yaml').write_text(
                f'image: projection.pgm\nmode: trinary\nresolution: {grid.info.resolution}\n'
                f'origin: [{origin.position.x}, {origin.position.y}, {grid_yaw}]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        (directory/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest, indent=2), flush=True)
finally:
    node.destroy_node()
    rclpy.try_shutdown()
