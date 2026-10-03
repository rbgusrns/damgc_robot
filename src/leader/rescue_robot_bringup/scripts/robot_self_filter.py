#!/usr/bin/env python3
"""Remove depth returns inside a conservative gripper swept volume."""

import numpy as np
import rclpy
import time
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
        self.declare_parameter("gripper_bounds_m", [0.08, 0.43, -0.20, 0.20, 0.00, 0.22])
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        # Depth callbacks may wait briefly for the image-time transform. Give
        # TF its own executor thread so those callbacks can keep filling the
        # buffer while the depth callback is waiting.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.camera_info = None
        self.camera_info_key = None
        self.filter_geometry = None
        self.last_warn = self.get_clock().now()
        self.stats_started = time.monotonic()
        self.stats_frames = 0
        self.stats_dropped = 0
        self.stats_removed = 0
        self.stats_valid_depth = 0
        self.stats_processing_ms = 0.0
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
        key = (msg.header.frame_id, msg.width, msg.height, tuple(msg.k))
        if key != self.camera_info_key:
            self.camera_info = msg
            self.camera_info_key = key
            self.filter_geometry = None

    @staticmethod
    def _quat_rotate(qx, qy, qz, qw, points):
        qv = np.array([qx, qy, qz], dtype=np.float32)
        v = points
        return v + 2.0 * np.cross(qv, np.cross(qv, v) + qw * v)

    def _prepare_filter_geometry(self, msg, tf, bounds):
        """Precompute per-pixel depth intervals for the fixed gripper volume."""
        info = self.camera_info
        xmin, xmax, ymin, ymax, zmin, zmax = bounds
        q, t = tf.transform.rotation, tf.transform.translation
        corners = np.array([
            [x, y, z] for x in (xmin, xmax) for y in (ymin, ymax) for z in (zmin, zmax)
        ], dtype=np.float32)
        camera_corners = self._quat_rotate(q.x, q.y, q.z, q.w, corners)
        camera_corners += np.array([t.x, t.y, t.z], dtype=np.float32)
        zc = camera_corners[:, 2]
        valid_corners = zc > 0.02
        if not np.any(valid_corners):
            raise ValueError("gripper swept volume is behind the depth camera")

        k = info.k
        u = k[0] * camera_corners[valid_corners, 0] / zc[valid_corners] + k[2]
        v = k[4] * camera_corners[valid_corners, 1] / zc[valid_corners] + k[5]
        u0 = min(msg.width, max(0, int(np.floor(np.min(u))) - 2))
        u1 = min(msg.width, max(0, int(np.ceil(np.max(u))) + 3))
        v0 = min(msg.height, max(0, int(np.floor(np.min(v))) - 2))
        v1 = min(msg.height, max(0, int(np.ceil(np.max(v))) + 3))
        if u1 <= u0 or v1 <= v0:
            return {"empty": True}

        # The camera-to-base transform is fixed. Convert each pixel ray once,
        # then intersect its depth with the three base_link box intervals.
        xx, yy = np.meshgrid(
            np.arange(u0, u1, dtype=np.float32),
            np.arange(v0, v1, dtype=np.float32),
        )
        camera_rays = np.stack((
            (xx - k[2]) / k[0],
            (yy - k[5]) / k[4],
            np.ones_like(xx),
        ), axis=-1)
        ray_base = self._quat_rotate(-q.x, -q.y, -q.z, q.w, camera_rays)
        offset = self._quat_rotate(
            -q.x, -q.y, -q.z, q.w,
            -np.array([t.x, t.y, t.z], dtype=np.float32),
        )

        depth_min = np.full(xx.shape, 0.02, dtype=np.float32)
        depth_max = np.full(xx.shape, np.inf, dtype=np.float32)
        valid = np.ones(xx.shape, dtype=bool)
        for axis, lower, upper in (
            (0, xmin, xmax), (1, ymin, ymax), (2, zmin, zmax)
        ):
            ray = ray_base[..., axis]
            parallel = np.abs(ray) < 1e-6
            inside_when_parallel = (offset[axis] >= lower) & (offset[axis] <= upper)
            valid &= ~(parallel & ~inside_when_parallel)
            moving = ~parallel
            first = np.zeros(xx.shape, dtype=np.float32)
            second = np.zeros(xx.shape, dtype=np.float32)
            np.divide(lower - offset[axis], ray, out=first, where=moving)
            np.divide(upper - offset[axis], ray, out=second, where=moving)
            depth_min = np.maximum(depth_min, np.minimum(first, second, where=moving,
                                                          out=np.full_like(first, -np.inf)))
            depth_max = np.minimum(depth_max, np.maximum(first, second, where=moving,
                                                          out=np.full_like(second, np.inf)))
        valid &= depth_max >= depth_min
        self.get_logger().info(
            f"Precomputed gripper filter ROI {u1 - u0}x{v1 - v0} "
            f"({100.0 * (u1 - u0) * (v1 - v0) / (msg.width * msg.height):.1f}% of frame)")
        return {
            "empty": False, "u0": u0, "u1": u1, "v0": v0, "v1": v1,
            "depth_min": depth_min, "depth_max": depth_max, "valid": valid,
        }

    def _depth_cb(self, msg):
        if not bool(self.get_parameter("enabled").value):
            self.pub.publish(msg)
            return
        if self.camera_info is None:
            self._warn_throttled("Self-filter waiting for camera_info; depth frame dropped")
            return
        if msg.encoding not in ("16UC1", "mono16", "32FC1"):
            self._warn_throttled(f"Unsupported depth encoding {msg.encoding}; frame dropped")
            return
        started = time.monotonic()
        try:
            info = self.camera_info
            bounds = np.asarray(self.get_parameter("gripper_bounds_m").value, dtype=np.float64)
            if bounds.size != 6:
                raise ValueError("gripper_bounds_m must contain xmin,xmax,ymin,ymax,zmin,zmax")
            # The camera is rigidly mounted on base_link. Resolve its static
            # transform once; per-frame TF and 3D point transforms are avoided.
            key = (msg.header.frame_id, msg.width, msg.height, tuple(info.k), tuple(bounds))
            if self.filter_geometry is None or self.filter_geometry.get("key") != key:
                tf = self.tf_buffer.lookup_transform(
                    msg.header.frame_id, str(self.get_parameter("base_frame").value),
                    rclpy.time.Time()
                )
                self.filter_geometry = self._prepare_filter_geometry(msg, tf, bounds)
                self.filter_geometry["key"] = key
            geometry = self.filter_geometry
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if geometry["empty"]:
                self.pub.publish(msg)
                return

            v0, v1 = geometry["v0"], geometry["v1"]
            u0, u1 = geometry["u0"], geometry["u1"]
            roi_depth = depth[v0:v1, u0:u1]
            if msg.encoding in ("16UC1", "mono16"):
                scale = 1000.0
                valid_depth = roi_depth > 20
            else:
                scale = 1.0
                valid_depth = roi_depth > 0.02
            mask = (
                geometry["valid"] & valid_depth
                & (roi_depth >= geometry["depth_min"] * scale)
                & (roi_depth <= geometry["depth_max"] * scale)
            )
            removed = int(np.count_nonzero(mask))
            valid_depth_count = int(np.count_nonzero(valid_depth))
            filtered = depth.copy()
            filtered[v0:v1, u0:u1][mask] = 0
            out = self.bridge.cv2_to_imgmsg(filtered, encoding=msg.encoding)
            out.header = msg.header
            self.pub.publish(out)
            self.stats_frames += 1
            self.stats_removed += removed
            self.stats_valid_depth += valid_depth_count
            self.stats_processing_ms += (time.monotonic() - started) * 1000.0
            elapsed = time.monotonic() - self.stats_started
            if elapsed >= 5.0:
                avg_ms = self.stats_processing_ms / max(1, self.stats_frames)
                removed_pct = 100.0 * self.stats_removed / max(1, self.stats_valid_depth)
                self.get_logger().info(
                    "5s filter health frames=%d dropped=%d mean=%.2fms "
                    "removed=%d valid_depth=%d removed_pct=%.2f%%" % (
                        self.stats_frames, self.stats_dropped, avg_ms,
                        self.stats_removed, self.stats_valid_depth, removed_pct))
                self.stats_started = time.monotonic()
                self.stats_frames = 0
                self.stats_dropped = 0
                self.stats_removed = 0
                self.stats_valid_depth = 0
                self.stats_processing_ms = 0.0
        except Exception as exc:
            self.stats_dropped += 1
            self._warn_throttled(f"Self-filter dropped frame: {exc}")
            # Never pass an unfiltered frame to nvblox: even one early frame
            # can leave a permanent self-obstacle in the static map.

    def _warn_throttled(self, message):
        now = self.get_clock().now()
        if (now - self.last_warn).nanoseconds > 5_000_000_000:
            self.get_logger().warning(message)
            self.last_warn = now


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
