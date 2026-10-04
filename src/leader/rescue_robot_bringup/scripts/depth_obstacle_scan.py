#!/usr/bin/env python3
"""Project sampled, height-filtered depth returns into a virtual base-frame scan."""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


def project_scan(depth, intrinsics, rotation, translation, stride=4,
                 min_height=0.01, max_height=0.30, min_range=0.20, max_range=4.0):
    """Return 360 one-degree bins; unobserved bins stay NaN, never free space."""
    rows, cols = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride].ravel()
    fx, fy, cx, cy = intrinsics
    points = np.column_stack(((cols.ravel()-cx)*z/fx,
                              (rows.ravel()-cy)*z/fy, z))
    valid = np.isfinite(z) & (z > 0) & (z <= max_range)
    points = points[valid] @ rotation.T + translation
    distance = np.hypot(points[:, 0], points[:, 1])
    # Conservative body/gripper envelope, including the gripper swept volume.
    own_body = ((points[:, 0] >= -0.08) & (points[:, 0] <= 0.38)
                & (np.abs(points[:, 1]) <= 0.16) & (points[:, 2] <= 0.16))
    keep = ((points[:, 2] >= min_height) & (points[:, 2] <= max_height)
            & (distance >= min_range) & (distance <= max_range) & ~own_body)
    angles = np.arctan2(points[keep, 1], points[keep, 0])
    indices = np.floor((angles + math.pi) / (math.pi / 180)).astype(int) % 360
    ranges = np.full(360, np.inf, dtype=np.float32)
    np.minimum.at(ranges, indices, distance[keep])
    ranges[np.isinf(ranges)] = np.nan
    return ranges


class DepthObstacleScan(Node):
    def __init__(self):
        super().__init__('depth_obstacle_scan')
        defaults = dict(depth_topic='/leader/camera/depth/image_rect_raw',
                        camera_info_topic='/leader/camera/depth/camera_info',
                        base_frame='base_link', pixel_stride=4, max_rate_hz=10.0,
                        min_height=0.01, max_height=0.30, min_range=0.20, max_range=4.0)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        if self.settings['pixel_stride'] < 1 or self.settings['max_rate_hz'] <= 0:
            raise ValueError('pixel_stride and max_rate_hz must be positive')
        self.info = None
        self.last = 0.0
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self, spin_thread=True)
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.settings['camera_info_topic'],
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.settings['depth_topic'],
                                 self.on_depth, qos_profile_sensor_data)

    def on_info(self, msg):
        self.info = msg

    def on_depth(self, msg):
        now = time.monotonic()
        if self.info is None or now-self.last < 1/self.settings['max_rate_hz']:
            return
        self.last = now
        try:
            if (msg.width, msg.height) != (self.info.width, self.info.height):
                raise ValueError('Depth and camera info dimensions differ')
            if msg.encoding not in ('16UC1', '32FC1'):
                raise ValueError(f'Unsupported depth encoding: {msg.encoding}')
            dtype = np.dtype(('>' if msg.is_bigendian else '<')
                             + ('u2' if msg.encoding == '16UC1' else 'f4'))
            depth = np.ndarray((msg.height, msg.width), dtype=dtype,
                               buffer=msg.data, strides=(msg.step, dtype.itemsize)).astype(np.float32)
            if msg.encoding == '16UC1':
                depth *= 0.001
            transform = self.tf.lookup_transform(self.settings['base_frame'],
                        msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp))
            q = transform.transform.rotation
            x,y,z,w = q.x,q.y,q.z,q.w
            rotation = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                                 [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                                 [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
            t = transform.transform.translation
            p = self.info.p
            intrinsics = (p[0],p[5],p[2],p[6]) if p[0] > 0 else (
                self.info.k[0],self.info.k[4],self.info.k[2],self.info.k[5])
            scan = LaserScan()
            scan.header.stamp = msg.header.stamp
            scan.header.frame_id = self.settings['base_frame']
            scan.angle_min = -math.pi
            scan.angle_increment = math.pi/180
            scan.angle_max = scan.angle_min+359*scan.angle_increment
            scan.scan_time = 1/self.settings['max_rate_hz']
            scan.range_min = self.settings['min_range']
            scan.range_max = self.settings['max_range']
            scan.ranges = project_scan(depth, intrinsics, rotation, np.array([t.x,t.y,t.z]),
                stride=self.settings['pixel_stride'], **{key:self.settings[key] for key in
                ('min_height','max_height','min_range','max_range')}).tolist()
            self.pub.publish(scan)
        except (TransformException, ValueError) as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = DepthObstacleScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
