#!/usr/bin/env python3
"""ROS2 open-loop gripper controller for RX-64/RX-28 through U2D2.

Command topic: /<namespace>/dynamixel/command
Message: std_msgs/msg/Float64MultiArray, data=[rx64_raw, rx28_raw, torque]

Use -1 for a position or torque field that must remain unchanged.
Example: [420.0, 1.0, 1.0] enables torque and sends both positions.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from dynamixel_orin import DynamixelOrin, get_profile


class DynamixelOrinNode(Node):
    def __init__(self):
        super().__init__("dynamixel_orin_node")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("robot", "leader")

        robot = str(self.get_parameter("robot").value)
        self.profile = get_profile(robot)
        port = str(self.get_parameter("port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        self.controller = None

        try:
            self.controller = DynamixelOrin(port, self.profile, baudrate)
            self.publish_status(f"READY robot={robot} port={port} baudrate={baudrate}")
        except Exception as exc:
            self.publish_status(f"ERROR opening U2D2: {exc}")
            self.get_logger().error(str(exc))

        self.subscription = self.create_subscription(
            Float64MultiArray,
            "dynamixel/command",
            self.command_callback,
            10,
        )
        self.status_publisher = self.create_publisher(String, "dynamixel/status", 10)

    def publish_status(self, message: str):
        self.get_logger().info(message)
        if hasattr(self, "status_publisher"):
            self.status_publisher.publish(String(data=message))

    def command_callback(self, message: Float64MultiArray):
        if self.controller is None:
            self.publish_status("ERROR controller is not connected")
            return
        if len(message.data) < 3:
            self.publish_status("ERROR command must be [rx64_raw, rx28_raw, torque]")
            return

        rx64_raw, rx28_raw, torque = message.data[:3]
        if not all(math.isfinite(value) for value in (rx64_raw, rx28_raw, torque)):
            self.publish_status("ERROR command contains NaN or infinity")
            return

        try:
            if torque >= 0:
                enabled = bool(round(torque))
                self.controller.set_torque(self.profile["rx64_id"], enabled)
                self.controller.set_torque(self.profile["rx28_id"], enabled)

            if rx64_raw >= 0:
                self.controller.set_position(
                    self.profile["rx64_id"],
                    round(rx64_raw),
                    self.profile["rx64_min"],
                    self.profile["rx64_max"],
                )

            if rx28_raw >= 0:
                self.controller.set_position(
                    self.profile["rx28_id"],
                    round(rx28_raw),
                    self.profile["rx28_min"],
                    self.profile["rx28_max"],
                )

            self.publish_status(
                f"OK rx64={round(rx64_raw)} rx28={round(rx28_raw)} torque={round(torque)}"
            )
        except Exception as exc:
            self.publish_status(f"ERROR command failed: {exc}")

    def destroy_node(self):
        if self.controller is not None:
            try:
                self.controller.set_torque(self.profile["rx64_id"], False)
                self.controller.set_torque(self.profile["rx28_id"], False)
            except Exception as exc:
                self.get_logger().error(f"failed to disable torque: {exc}")
            self.controller.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelOrinNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
