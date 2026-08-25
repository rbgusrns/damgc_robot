#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class VslamCovarianceAdapter(Node):
    """Add conservative covariance to the cuVSLAM map-pose output."""

    def __init__(self):
        super().__init__("vslam_covariance_adapter")

        self.declare_parameter("input_topic", "/visual_slam/vis/slam_odometry")
        self.declare_parameter(
            "output_topic",
            "/visual_slam/slam_odometry_with_covariance",
        )
        self.declare_parameter("position_stddev_m", 0.05)
        self.declare_parameter("yaw_stddev_rad", 0.05)
        self.declare_parameter("unmeasured_variance", 1000000.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._publisher = self.create_publisher(Odometry, output_topic, 10)
        self._subscription = self.create_subscription(
            Odometry,
            input_topic,
            self._callback,
            10,
        )

        self.get_logger().info(
            "Adding covariance to VSLAM map poses: "
            f"{input_topic} -> {output_topic}"
        )

    def _callback(self, message):
        output = deepcopy(message)
        position_variance = float(
            self.get_parameter("position_stddev_m").value
        ) ** 2
        yaw_variance = float(
            self.get_parameter("yaw_stddev_rad").value
        ) ** 2
        unmeasured_variance = float(
            self.get_parameter("unmeasured_variance").value
        )

        output.pose.covariance = [0.0] * 36
        output.pose.covariance[0] = position_variance
        output.pose.covariance[7] = position_variance
        output.pose.covariance[14] = unmeasured_variance
        output.pose.covariance[21] = unmeasured_variance
        output.pose.covariance[28] = unmeasured_variance
        output.pose.covariance[35] = yaw_variance

        # Twist from vis/slam_odometry is not populated and is not fused.
        output.twist.covariance = [0.0] * 36
        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = VslamCovarianceAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
