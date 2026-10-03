#!/usr/bin/env python3
"""Read RX-64/RX-28 present positions and publish them without writing registers."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from dynamixel_orin import DynamixelCommunicationError, DynamixelOrin, get_profile


class DynamixelStatePublisher(Node):
    def __init__(self):
        super().__init__("dynamixel_state_publisher")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("robot", "leader")
        self.declare_parameter("publish_rate", 2.0)
        self.robot = str(self.get_parameter("robot").value)
        self.profile = get_profile(self.robot)
        self.controller = DynamixelOrin(
            str(self.get_parameter("port").value),
            self.profile,
            int(self.get_parameter("baudrate").value),
        )
        self.position_pub = self.create_publisher(
            Float64MultiArray, f"/{self.robot}/dynamixel/present_position_raw", 10
        )
        self.status_pub = self.create_publisher(
            String, f"/{self.robot}/dynamixel/telemetry_status", 10
        )
        rate = float(self.get_parameter("publish_rate").value)
        if rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        self.timer = self.create_timer(1.0 / rate, self.publish_state)
        self.get_logger().info(
            "Read-only telemetry active; positions are [RX64, RX28] raw ticks"
        )

    def publish_state(self):
        values = []
        try:
            for device_id in (self.profile["rx64_id"], self.profile["rx28_id"]):
                value, result, error = self.controller.packet_handler.read2ByteTxRx(
                    self.controller.port_handler, device_id, 36
                )
                self.controller._check_result(
                    f"{self.controller._label(device_id)} present position",
                    result,
                    error,
                )
                values.append(float(value))
            self.position_pub.publish(Float64MultiArray(data=values))
            self.status_pub.publish(String(data="READY"))
        except Exception as exc:
            self.status_pub.publish(String(data=f"ERROR {exc}"))
            self.get_logger().warning(f"Dynamixel read failed: {exc}")

    def destroy_node(self):
        self.controller.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DynamixelStatePublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


from rclpy.executors import ExternalShutdownException


if __name__ == "__main__":
    main()
