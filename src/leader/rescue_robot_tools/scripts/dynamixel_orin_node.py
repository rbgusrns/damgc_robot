#!/usr/bin/env python3
"""ROS2 open-loop gripper controller for RX-64/RX-28 through U2D2.

Raw command topic: /<namespace>/dynamixel/command
Message: std_msgs/msg/Float64MultiArray, data=[rx64_raw, rx28_raw, torque]

Semantic command topic: /<namespace>/gripper/command
Message: std_msgs/msg/String, data=open|close|middle|rx64_high|rx64_low|rx64_middle|stop

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
        self.gripper_subscription = self.create_subscription(
            String,
            "gripper/command",
            self.gripper_command_callback,
            10,
        )
        self.status_publisher = self.create_publisher(String, "dynamixel/status", 10)

    def gripper_command_callback(self, message: String):
        """Handle simple semantic commands without exposing raw positions."""
        if self.controller is None:
            self.publish_status("ERROR controller is not connected")
            return

        command = message.data.strip().lower()
        rx28_positions = {
            "open": self.profile["rx28_max"],
            "close": self.profile["rx28_min"],
            "middle": round((self.profile["rx28_min"] + self.profile["rx28_max"]) / 2),
        }
        rx64_positions = {
            "rx64_high": self.profile["rx64_min"],
            "rx64_low": self.profile["rx64_max"],
            "rx64_middle": round((self.profile["rx64_min"] + self.profile["rx64_max"]) / 2),
        }

        try:
            if command in rx28_positions:
                self.controller.set_torque(self.profile["rx28_id"], True)
                self.controller.set_position(
                    self.profile["rx28_id"],
                    rx28_positions[command],
                    self.profile["rx28_min"],
                    self.profile["rx28_max"],
                )
                self.publish_status(f"OK command={command} rx28={rx28_positions[command]}")
            elif command in rx64_positions:
                self.controller.set_torque(self.profile["rx64_id"], True)
                self.controller.set_position(
                    self.profile["rx64_id"],
                    rx64_positions[command],
                    self.profile["rx64_min"],
                    self.profile["rx64_max"],
                )
                self.publish_status(f"OK command={command} rx64={rx64_positions[command]}")
            elif command == "stop":
                self.controller.set_torque(self.profile["rx64_id"], False)
                self.controller.set_torque(self.profile["rx28_id"], False)
                self.publish_status("OK command=stop torque=0")
            else:
                self.publish_status(
                    "ERROR command must be open, close, middle, "
                    "rx64_high, rx64_low, rx64_middle, or stop"
                )
        except Exception as exc:
            self.publish_status(f"ERROR command failed: {exc}")

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
