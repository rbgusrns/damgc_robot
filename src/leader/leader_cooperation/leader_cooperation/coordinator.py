"""Leader-side DDS bridge for cooperative transport.

The node deliberately uses only standard ROS messages so the leader and follower
can be built independently. A follower status message is treated as a heartbeat;
commands are forwarded only while cooperation is explicitly started.
"""

from enum import Enum
from math import isfinite
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool


class CooperationState(str, Enum):
    IDLE = "IDLE"
    WAITING_FOLLOWER = "WAITING_FOLLOWER"
    COOPERATING = "COOPERATING"
    FAULT = "FAULT"


class LeaderCooperationNode(Node):
    """Forward leader velocity to the follower with explicit safety gates."""

    def __init__(self) -> None:
        super().__init__("leader_cooperation")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("follower_status_timeout", 1.0)
        self.declare_parameter("max_linear_x", 0.25)
        self.declare_parameter("max_angular_z", 1.0)

        self._rate = float(self.get_parameter("publish_rate").value)
        self._command_timeout = float(self.get_parameter("command_timeout").value)
        self._status_timeout = float(self.get_parameter("follower_status_timeout").value)
        self._max_linear_x = abs(float(self.get_parameter("max_linear_x").value))
        self._max_angular_z = abs(float(self.get_parameter("max_angular_z").value))
        if not all(isfinite(v) and v > 0.0 for v in (
            self._rate, self._command_timeout, self._status_timeout,
            self._max_linear_x, self._max_angular_z,
        )):
            raise ValueError("cooperation safety parameters must be finite and positive")

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_pub = self.create_publisher(String, "/cooperation/state", state_qos)
        self._mission_pub = self.create_publisher(String, "/mission/state", state_qos)
        self._target_pub = self.create_publisher(Twist, "/cooperation/target_velocity", reliable)
        self._follower_cmd_pub = self.create_publisher(Twist, "/follower/cmd_vel", reliable)
        self._leader_cmd_sub = self.create_subscription(
            Twist, "/leader/cmd_vel", self._on_leader_cmd, reliable
        )
        self._status_sub = self.create_subscription(
            String, "/follower/status", self._on_follower_status, reliable
        )
        self._control_srv = self.create_service(SetBool, "/cooperation/enable", self._on_enable)

        self._last_command = Twist()
        self._last_command_time: Optional[float] = None
        self._last_status_time: Optional[float] = None
        self._requested = False
        self._state = CooperationState.IDLE
        self._timer = self.create_timer(1.0 / self._rate, self._on_timer)
        self._publish_state()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def _on_leader_cmd(self, message: Twist) -> None:
        command = Twist()
        command.linear.x = max(-self._max_linear_x, min(self._max_linear_x, message.linear.x))
        command.angular.z = max(-self._max_angular_z, min(self._max_angular_z, message.angular.z))
        self._last_command = command
        self._last_command_time = self._now()

    def _on_follower_status(self, _message: String) -> None:
        self._last_status_time = self._now()

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response):
        self._requested = bool(request.data)
        if self._requested:
            self._state = CooperationState.WAITING_FOLLOWER
            response.message = "cooperation requested; waiting for follower heartbeat"
        else:
            self._state = CooperationState.IDLE
            response.message = "cooperation stopped"
        response.success = True
        self._publish_state()
        return response

    def _status_is_fresh(self, now: float) -> bool:
        return self._last_status_time is not None and now - self._last_status_time <= self._status_timeout

    def _command_is_fresh(self, now: float) -> bool:
        return self._last_command_time is not None and now - self._last_command_time <= self._command_timeout

    @staticmethod
    def _zero() -> Twist:
        return Twist()

    def _on_timer(self) -> None:
        now = self._now()
        if not self._requested:
            self._state = CooperationState.IDLE
            command = self._zero()
        elif not self._status_is_fresh(now):
            if self._state == CooperationState.COOPERATING:
                self.get_logger().error("follower heartbeat lost; stopping cooperation")
                self._state = CooperationState.FAULT
            elif self._state != CooperationState.FAULT:
                self._state = CooperationState.WAITING_FOLLOWER
            command = self._zero()
        elif self._state == CooperationState.FAULT:
            command = self._zero()
        else:
            self._state = CooperationState.COOPERATING
            command = self._last_command if self._command_is_fresh(now) else self._zero()
        self._target_pub.publish(command)
        self._follower_cmd_pub.publish(command)
        self._publish_state()

    def _publish_state(self) -> None:
        state = String(data=self._state.value)
        self._state_pub.publish(state)
        self._mission_pub.publish(String(data="COOPERATIVE_TRANSPORT" if self._state == CooperationState.COOPERATING else self._state.value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeaderCooperationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._follower_cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
