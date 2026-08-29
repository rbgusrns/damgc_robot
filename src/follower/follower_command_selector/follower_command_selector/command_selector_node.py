"""Select exactly one explicit fresh Follower velocity command source."""

import time
from math import isfinite
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from follower_command_selector.command_selector_logic import (
    CommandSource,
    PlanarCommand,
    SelectorParameters,
    sanitize_command,
    select_command,
)


COMMAND_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class CommandSelectorNode(Node):
    """Enforce explicit command ownership with post-switch freshness."""

    def __init__(self) -> None:
        super().__init__("command_selector")
        self._declare_parameters()
        self._load_and_validate_parameters()

        self._selected_pub = self.create_publisher(
            Twist, "selected_cmd_vel", COMMAND_QOS
        )
        self._approach_sub = self.create_subscription(
            Twist,
            "approach/cmd_vel_raw",
            self._on_approach_command,
            COMMAND_QOS,
        )
        self._cooperation_sub = self.create_subscription(
            Twist, "cmd_vel", self._on_cooperation_command, COMMAND_QOS
        )

        self._approach_command: Optional[PlanarCommand] = None
        self._approach_received_seconds: Optional[float] = None
        self._cooperation_command: Optional[PlanarCommand] = None
        self._cooperation_received_seconds: Optional[float] = None
        self._parameter_callback = self.add_on_set_parameters_callback(
            self._on_parameter_change
        )
        self._timer = self.create_timer(1.0 / self._publish_rate, self._on_timer)
        self.get_logger().info(
            "Command selector ready: source=%s, timeouts=(approach %.3fs, "
            "cooperation %.3fs)"
            % (
                self._source.value,
                self._selector_parameters.approach_timeout,
                self._selector_parameters.cooperation_timeout,
            )
        )

    def _declare_parameters(self) -> None:
        """Declare startup and runtime source configuration."""
        self.declare_parameter("source_mode", CommandSource.STOP.value)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("approach_timeout", 0.35)
        self.declare_parameter("cooperation_timeout", 0.50)
        self.declare_parameter("axis_epsilon", 1.0e-9)

    def _load_and_validate_parameters(self) -> None:
        """Load startup settings and reject ambiguous values."""
        source_value = str(self.get_parameter("source_mode").value)
        try:
            self._source = CommandSource(source_value)
        except ValueError as error:
            raise ValueError(
                "source_mode must be STOP, APPROACH, or COOPERATION"
            ) from error
        self._publish_rate = float(self.get_parameter("publish_rate").value)
        self._selector_parameters = SelectorParameters(
            approach_timeout=float(self.get_parameter("approach_timeout").value),
            cooperation_timeout=float(
                self.get_parameter("cooperation_timeout").value
            ),
            axis_epsilon=float(self.get_parameter("axis_epsilon").value),
        )
        if not isfinite(self._publish_rate) or self._publish_rate <= 0.0:
            raise ValueError("publish_rate must be finite and greater than zero")
        self._selector_parameters.validate()

    def _on_approach_command(self, message: Twist) -> None:
        """Cache only a valid command from the currently selected source."""
        if self._source != CommandSource.APPROACH:
            return
        self._approach_command = self._sanitize(message)
        self._approach_received_seconds = (
            time.monotonic() if self._approach_command is not None else None
        )
        if self._approach_command is None:
            self._selected_pub.publish(Twist())

    def _on_cooperation_command(self, message: Twist) -> None:
        """Cache only a valid command from the currently selected source."""
        if self._source != CommandSource.COOPERATION:
            return
        self._cooperation_command = self._sanitize(message)
        self._cooperation_received_seconds = (
            time.monotonic() if self._cooperation_command is not None else None
        )
        if self._cooperation_command is None:
            self._selected_pub.publish(Twist())

    def _sanitize(self, message: Twist) -> Optional[PlanarCommand]:
        """Validate all six Twist axes at the selector input boundary."""
        return sanitize_command(
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
            self._selector_parameters.axis_epsilon,
        )

    def _on_parameter_change(
        self, parameters: List[Parameter]
    ) -> SetParametersResult:
        """Accept only an explicit valid source change at runtime."""
        if len(parameters) != 1 or parameters[0].name != "source_mode":
            return SetParametersResult(
                successful=False,
                reason="only source_mode may be changed at runtime",
            )
        parameter = parameters[0]
        if parameter.type_ != Parameter.Type.STRING:
            return SetParametersResult(
                successful=False, reason="source_mode must be a string"
            )
        try:
            source = CommandSource(str(parameter.value))
        except ValueError:
            return SetParametersResult(
                successful=False,
                reason="source_mode must be STOP, APPROACH, or COOPERATION",
            )

        if source != self._source:
            self._source = source
            self._clear_commands()
            self._selected_pub.publish(Twist())
            self.get_logger().info(
                "Command source changed to %s; waiting for a fresh command"
                % source.value
            )
        return SetParametersResult(successful=True)

    def _on_timer(self) -> None:
        """Publish the selected fresh command or an explicit zero."""
        command = select_command(
            self._source,
            time.monotonic(),
            self._selector_parameters,
            approach_command=self._approach_command,
            approach_received_seconds=self._approach_received_seconds,
            cooperation_command=self._cooperation_command,
            cooperation_received_seconds=self._cooperation_received_seconds,
        )
        self._selected_pub.publish(self._to_twist(command))

    def _clear_commands(self) -> None:
        """Discard every source cache across ownership transitions."""
        self._approach_command = None
        self._approach_received_seconds = None
        self._cooperation_command = None
        self._cooperation_received_seconds = None

    @staticmethod
    def _to_twist(command: PlanarCommand) -> Twist:
        """Populate only the validated differential-drive axes."""
        message = Twist()
        message.linear.x = command.linear_x
        message.angular.z = command.angular_z
        return message

    def stop(self) -> None:
        """Publish explicit zero before process shutdown."""
        self._selected_pub.publish(Twist())


def main(args: Optional[List[str]] = None) -> None:
    """Initialize ROS 2 and spin the command selector."""
    rclpy.init(args=args)
    node: Optional[CommandSelectorNode] = None
    try:
        node = CommandSelectorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                if rclpy.ok():
                    node.stop()
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


if __name__ == "__main__":
    main()
