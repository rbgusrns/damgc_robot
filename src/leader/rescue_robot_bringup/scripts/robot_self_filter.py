#!/usr/bin/env python3
"""Remove depth returns inside a conservative gripper swept volume."""

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


class RobotSelfFilter(Node):
    def __init__(self):
        super().__init__("robot_self_filter")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("depth_topic", "/leader/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/leader/camera/depth/camera_info")
        self.declare_parameter("filtered_topic", "/leader/camera/depth/self_filtered")
        self.declare_parameter("enabled", True)
        # [xmin, xmax, ymin, ymax, zmin, zmax] in base_link, metres.
        # This envelope includes fixed gripper geometry plus opening/lift sway.
        # Returns inside it are intentionally removed, including nearby objects.
        self.declare_parameter("gripper_bounds_m", [0.12, 0.38, -0.15, 0.15, 0.00, 0.16])
        self._holding = False
        self.create_subscription(String, "/leader/mapping/mode", self._mode_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        # Depth callbacks may wait briefly for the image-time transform. Give
        # TF its own executor thread so those callbacks can keep filling the
        # buffer while the depth callback is waiting.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.camera_info = None
        self.last_warn = self.get_clock().now()
        self.pub = self.create_publisher(
            Image, str(self.get_parameter("filtered_topic").value), qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value),
            self._camera_info_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, str(self.get_parameter("depth_topic").value),
            self._depth_cb, qos_profile_sensor_data
        )
        self.get_logger().info("Robot gripper depth self-filter ready")

    def _camera_info_cb(self, msg):
        self.camera_info = msg

    @staticmethod
    def _quat_rotate(qx, qy, qz, qw, points):
        qv = np.array([qx, qy, qz], dtype=np.float64)
        v = points
        return v + 2.0 * np.cross(qv, np.cross(qv, v) + qw * v)

    @staticmethod
    def _rotation_matrix(qx, qy, qz, qw):
        return np.array([
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw),
             2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz),
             2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw),
             1.0 - 2.0 * (qx * qx + qy * qy)],
        ], dtype=np.float32)

    def _prepare_filter_geometry(self, msg, info, bounds):
        key = (
            msg.width, msg.height, msg.header.frame_id, msg.encoding,
            tuple(float(value) for value in info.k), tuple(float(value) for value in bounds),
        )
        if key == getattr(self, "_geometry_key", None):
            return self._filter_geometry

        xmin, xmax, ymin, ymax, zmin, zmax = bounds
        tf = self.tf_buffer.lookup_transform(
            msg.header.frame_id,
            str(self.get_parameter("base_frame").value),
            rclpy.time.Time(),
        )
        q, t = tf.transform.rotation, tf.transform.translation
        corners = np.array([
            [x, y, z]
            for x in (xmin, xmax)
            for y in (ymin, ymax)
            for z in (zmin, zmax)
        ], dtype=np.float32)
        camera_corners = self._quat_rotate(q.x, q.y, q.z, q.w, corners)
        camera_translation = np.array([t.x, t.y, t.z], dtype=np.float32)
        camera_corners += camera_translation
        zc = camera_corners[:, 2]
        valid_corners = zc > 0.02
        if not np.any(valid_corners):
            raise ValueError("gripper swept volume is behind the depth camera")

        k = info.k
        projected_u = k[0] * camera_corners[valid_corners, 0] / zc[valid_corners] + k[2]
        projected_v = k[4] * camera_corners[valid_corners, 1] / zc[valid_corners] + k[5]
        u0 = min(msg.width, max(0, int(np.floor(np.min(projected_u))) - 2))
        u1 = min(msg.width, max(0, int(np.ceil(np.max(projected_u))) + 3))
        v0 = min(msg.height, max(0, int(np.floor(np.min(projected_v))) - 2))
        v1 = min(msg.height, max(0, int(np.ceil(np.max(projected_v))) + 3))
        if u1 <= u0 or v1 <= v0:
            geometry = {"roi": (u0, u1, v0, v1)}
            self._geometry_key = key
            self._filter_geometry = geometry
            return geometry

        # Precompute where each ROI pixel ray intersects the fixed gripper box.
        # Depth values are camera Z, so rays use a Z component of exactly one.
        pixel_u, pixel_v = np.meshgrid(
            np.arange(u0, u1, dtype=np.float32),
            np.arange(v0, v1, dtype=np.float32),
        )
        rays_camera = np.stack((
            (pixel_u - np.float32(k[2])) / np.float32(k[0]),
            (pixel_v - np.float32(k[5])) / np.float32(k[4]),
            np.ones_like(pixel_u),
        ), axis=-1)

        rotation_camera_base = self._rotation_matrix(q.x, q.y, q.z, q.w)
        rotation_base_camera = rotation_camera_base.T
        camera_origin_base = -(rotation_base_camera @ camera_translation)
        # For row vectors, camera rays map back to base with the matrix that
        # maps base column vectors into camera coordinates.
        rays_base = rays_camera @ rotation_camera_base
        bounds_min = np.array([xmin, ymin, zmin], dtype=np.float32)
        bounds_max = np.array([xmax, ymax, zmax], dtype=np.float32)

        parallel = np.abs(rays_base) < 1.0e-7
        origin_inside = (camera_origin_base >= bounds_min) & (camera_origin_base <= bounds_max)
        parallel_inside = np.all(~parallel | origin_inside, axis=-1)
        with np.errstate(divide="ignore", invalid="ignore"):
            first = np.divide(
                bounds_min - camera_origin_base,
                rays_base,
                out=np.full_like(rays_base, -np.inf),
                where=~parallel,
            )
            second = np.divide(
                bounds_max - camera_origin_base,
                rays_base,
                out=np.full_like(rays_base, np.inf),
                where=~parallel,
            )
        entry = np.maximum(np.max(np.minimum(first, second), axis=-1), 0.02)
        exit_distance = np.min(np.maximum(first, second), axis=-1)
        intersects = parallel_inside & (exit_distance >= entry)

        scale = 1000.0 if msg.encoding in ("16UC1", "mono16") else 1.0
        geometry = {
            "roi": (u0, u1, v0, v1),
            "intersects": intersects,
            "minimum_depth": entry * scale,
            "maximum_depth": exit_distance * scale,
            "scale": scale,
        }
        self._geometry_key = key
        self._filter_geometry = geometry
        self.get_logger().info(
            f"Cached self-filter rays: ROI={u1 - u0}x{v1 - v0}, "
            f"active pixels={int(np.count_nonzero(intersects))}"
        )
        return geometry

    def _mode_cb(self, message):
        self._holding = message.data in ("HOLD", "LOADING")

    def _depth_cb(self, msg):
        if self._holding:
            return  # Do not integrate the carried object into the environment.
        if not bool(self.get_parameter("enabled").value):
            self.pub.publish(msg)
            return
        if self.camera_info is None or msg.encoding not in ("16UC1", "mono16", "32FC1"):
            return
        try:
            info = self.camera_info
            bounds = np.asarray(self.get_parameter("gripper_bounds_m").value, dtype=np.float64)
            if bounds.size != 6:
                raise ValueError("gripper_bounds_m must contain xmin,xmax,ymin,ymax,zmin,zmax")
            geometry = self._prepare_filter_geometry(msg, info, bounds)
            u0, u1, v0, v1 = geometry["roi"]
            if u1 <= u0 or v1 <= v0:
                self.pub.publish(msg)
                return

            if msg.encoding in ("16UC1", "mono16"):
                dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
            else:
                dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
            row_stride = msg.step // dtype.itemsize
            depth = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_stride)
            depth_roi = depth[v0:v1, u0:u1]
            mask = (
                geometry["intersects"]
                & (depth_roi > 0)
                & (depth_roi >= geometry["minimum_depth"])
                & (depth_roi <= geometry["maximum_depth"])
            )
            if np.any(mask):
                # This callback owns the received ROS message. Mutating its
                # buffer avoids a full-frame copy at camera rate.
                if not depth.flags.writeable:
                    msg.data = bytearray(msg.data)
                    depth = np.frombuffer(msg.data, dtype=dtype).reshape(
                        msg.height, row_stride
                    )
                    depth_roi = depth[v0:v1, u0:u1]
                depth_roi[mask] = 0
            self.pub.publish(msg)
        except Exception as exc:
            now = self.get_clock().now()
            if (now - self.last_warn).nanoseconds > 5_000_000_000:
                self.get_logger().warning(f"Self-filter dropped frame: {exc}")
                self.last_warn = now
            # Never pass an unfiltered frame to nvblox: even one early frame
            # can leave a permanent self-obstacle in the static map.


def main():
    rclpy.init()
    node = RobotSelfFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
