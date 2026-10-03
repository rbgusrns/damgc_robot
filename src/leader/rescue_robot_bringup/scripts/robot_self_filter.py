#!/usr/bin/env python3
"""Remove depth returns inside a conservative gripper swept volume."""

import numpy as np
import rclpy
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
        self.declare_parameter("gripper_bounds_m", [0.12, 0.38, -0.15, 0.15, 0.00, 0.16])
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

    def _depth_cb(self, msg):
        if not bool(self.get_parameter("enabled").value):
            self.pub.publish(msg)
            return
        if self.camera_info is None or msg.encoding not in ("16UC1", "mono16", "32FC1"):
            return
        try:
            info = self.camera_info
            # The camera is rigidly mounted on base_link. Use the latest static
            # transform; depth timestamps are ahead of TF timestamps on this rig.
            tf = self.tf_buffer.lookup_transform(
                msg.header.frame_id, str(self.get_parameter("base_frame").value),
                rclpy.time.Time()
            )
            bounds = np.asarray(self.get_parameter("gripper_bounds_m").value, dtype=np.float64)
            if bounds.size != 6:
                raise ValueError("gripper_bounds_m must contain xmin,xmax,ymin,ymax,zmin,zmax")
            xmin, xmax, ymin, ymax, zmin, zmax = bounds
            q, t = tf.transform.rotation, tf.transform.translation

            # Project the swept-volume corners to limit per-frame work to the
            # image region that can contain a gripper return.
            corners = np.array([
                [x, y, z] for x in (xmin, xmax) for y in (ymin, ymax) for z in (zmin, zmax)
            ], dtype=np.float64)
            camera_corners = self._quat_rotate(q.x, q.y, q.z, q.w, corners)
            camera_corners += np.array([t.x, t.y, t.z])
            zc = camera_corners[:, 2]
            valid_corners = zc > 0.02
            if not np.any(valid_corners):
                raise ValueError("gripper swept volume is behind the depth camera")
            k = info.k
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if msg.encoding in ("16UC1", "mono16"):
                measured = depth.astype(np.float32) * 0.001
            else:
                measured = depth.astype(np.float32)

            u = k[0] * camera_corners[valid_corners, 0] / zc[valid_corners] + k[2]
            v = k[4] * camera_corners[valid_corners, 1] / zc[valid_corners] + k[5]
            u0 = min(msg.width, max(0, int(np.floor(np.min(u))) - 2))
            u1 = min(msg.width, max(0, int(np.ceil(np.max(u))) + 3))
            v0 = min(msg.height, max(0, int(np.floor(np.min(v))) - 2))
            v1 = min(msg.height, max(0, int(np.ceil(np.max(v))) + 3))
            if u1 <= u0 or v1 <= v0:
                self.pub.publish(msg)
                return

            yy, xx = np.indices((v1 - v0, u1 - u0), dtype=np.float64)
            xx += u0
            yy += v0
            roi_depth = measured[v0:v1, u0:u1]
            camera_points = np.stack((
                (xx - k[2]) * roi_depth / k[0],
                (yy - k[5]) * roi_depth / k[4],
                roi_depth,
            ), axis=-1)
            translation = np.array([t.x, t.y, t.z])
            base_points = self._quat_rotate(
                -q.x, -q.y, -q.z, q.w, camera_points - translation
            )
            mask = (
                (roi_depth > 0.02)
                & (base_points[..., 0] >= xmin) & (base_points[..., 0] <= xmax)
                & (base_points[..., 1] >= ymin) & (base_points[..., 1] <= ymax)
                & (base_points[..., 2] >= zmin) & (base_points[..., 2] <= zmax)
            )
            filtered = depth.copy()
            filtered_roi = filtered[v0:v1, u0:u1]
            filtered_roi[mask] = 0
            out = self.bridge.cv2_to_imgmsg(filtered, encoding=msg.encoding)
            out.header = msg.header
            self.pub.publish(out)
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
