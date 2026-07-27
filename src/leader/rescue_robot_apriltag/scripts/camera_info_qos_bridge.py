#!/usr/bin/env python3
"""Republish D435 CameraInfo with transient-local durability for image_proc."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo


class CameraInfoQosBridge(Node):
    def __init__(self):
        super().__init__("camera_info_qos_bridge")
        input_topic = self.declare_parameter(
            "input_topic", "/leader/camera/color/camera_info"
        ).value
        output_topic = self.declare_parameter(
            "output_topic", "/leader/camera/color/camera_info_transient"
        ).value
        input_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(CameraInfo, output_topic, output_qos)
        self.subscription = self.create_subscription(
            CameraInfo, input_topic, self._callback, input_qos
        )
        self.get_logger().info(f"Bridging {input_topic} -> {output_topic}")

    def _callback(self, message):
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = CameraInfoQosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
