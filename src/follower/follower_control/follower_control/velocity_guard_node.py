"""Forward cooperation velocity with limits and a local timeout watchdog."""

from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from follower_control.velocity_guard_logic import command_is_fresh, sanitize_velocity


COMMAND_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class VelocityGuardNode(Node):
    """Provide the follower-side fail-safe boundary for remote velocity."""

    def __init__(self) -> None:
        super().__init__("velocity_guard")
        self.declare_parameter("command_timeout", 0.3)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("command_topic", "/follower/cmd_vel")
        self.declare_parameter("safe_command_topic", "/follower/safe_cmd_vel")

        self._timeout = float(self.get_parameter("command_timeout").value)
        self._publish_rate = float(self.get_parameter("publish_rate").value)
        self._max_linear = float(self.get_parameter("max_linear_speed").value)
        self._max_angular = float(self.get_parameter("max_angular_speed").value)
        self._command_topic = str(self.get_parameter("command_topic").value)
        self._safe_command_topic = str(
            self.get_parameter("safe_command_topic").value
        )
        if self._timeout <= 0.0:
            raise ValueError("command_timeout must be greater than zero")
        if self._publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")
        # Validate limits before accepting any command.
        sanitize_velocity(0.0, 0.0, self._max_linear, self._max_angular)

        self._last_command = Twist()
        self._last_received_seconds: Optional[float] = None
        self._connected: Optional[bool] = None

        self._cmd_pub = self.create_publisher(
            Twist, self._safe_command_topic, COMMAND_QOS
        )
        self._connected_pub = self.create_publisher(
            Bool, "/follower/command_connected", 1
        )
        self._status_pub = self.create_publisher(String, "/follower/status", 10)
        self._subscription = self.create_subscription(
            Twist,
            self._command_topic,
            self._on_command,
            COMMAND_QOS,
        )
        self._timer = self.create_timer(1.0 / self._publish_rate, self._on_timer)
        self.get_logger().info(
            "Velocity guard ready: timeout=%.3fs, limits=(%.3fm/s, %.3frad/s)"
            % (self._timeout, self._max_linear, self._max_angular)
        )

    def _on_command(self, message: Twist) -> None:
        now = self.get_clock().now().nanoseconds / 1.0e9
        try:
            safe = sanitize_velocity(
                message.linear.x,
                message.angular.z,
                self._max_linear,
                self._max_angular,
            )
        except ValueError as error:
            self.get_logger().error("Rejected unsafe velocity: %s" % error)
            self._last_received_seconds = None
            return

        command = Twist()
        command.linear.x = safe.linear_x
        command.angular.z = safe.angular_z
        self._last_command = command
        self._last_received_seconds = now

    def _on_timer(self) -> None:
        now = self.get_clock().now().nanoseconds / 1.0e9
        fresh = (
            self._last_received_seconds is not None
            and command_is_fresh(now, self._last_received_seconds, self._timeout)
        )
        self._cmd_pub.publish(self._last_command if fresh else Twist())
        if fresh != self._connected:
            self._connected = fresh
            state = "COMMAND_ACTIVE" if fresh else "COMMAND_TIMEOUT_STOPPED"
            self._connected_pub.publish(Bool(data=fresh))
            # Keep one severity for this logger.  rclpy rejects changing a
            # logger's severity between calls within the same process.
            self.get_logger().info(state)
        # Heartbeat is intentionally periodic, including while IDLE/timeout.
        self._status_pub.publish(String(data="READY" if not fresh else "ACTIVE"))

    def stop(self) -> None:
        """Publish several stop samples before shutdown."""
        stop = Twist()
        for _ in range(3):
            self._cmd_pub.publish(stop)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[VelocityGuardNode] = None
    try:
        node = VelocityGuardNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
