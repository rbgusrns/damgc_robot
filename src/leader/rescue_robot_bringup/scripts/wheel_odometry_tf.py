#!/usr/bin/env python3
"""Make wheel odometry the single pose authority for local mapping/navigation."""

from math import isfinite

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def wheel_transform(message):
    """Preserve the measured wheel pose and timestamp, rejecting invalid frames."""
    p, q = message.pose.pose.position, message.pose.pose.orientation
    norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    if (message.header.frame_id != "odom" or message.child_frame_id != "base_link"
            or not all(isfinite(v) for v in (p.x, p.y, p.z, norm))
            or abs(norm - 1.0) > 0.01):
        raise ValueError("Expected finite wheel pose in odom -> base_link")
    result = TransformStamped()
    result.header = message.header
    result.child_frame_id = message.child_frame_id
    result.transform.translation.x = p.x
    result.transform.translation.y = p.y
    result.transform.translation.z = p.z
    result.transform.rotation = q
    return result


class WheelOdometryTF(Node):
    def __init__(self):
        super().__init__("wheel_odometry_tf")
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/leader/odom/raw", self.on_odom,
                                 qos_profile_sensor_data)

    def on_odom(self, message):
        try:
            transform = wheel_transform(message)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=2.0)
            return
        age = self.get_clock().now().nanoseconds*1e-9 - (
            message.header.stamp.sec + message.header.stamp.nanosec*1e-9)
        if -0.1 <= age <= 0.5:
            self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = WheelOdometryTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
