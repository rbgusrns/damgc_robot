"""Select exactly one explicit fresh Leader velocity command source."""

import time
from math import isfinite
from typing import Dict, List, Optional

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from leader_command_selector.command_selector_logic import (
    CommandSource,
    PlanarCommand,
    SelectorParameters,
    sanitize_command,
    select_command,
    selection_status,
)


COMMAND_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
STATUS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class CommandSelectorNode(Node):
    """Enforce explicit Leader command ownership with post-switch freshness."""

    def __init__(self) -> None:
        super().__init__("command_selector")
        self._declare_parameters()
        self._load_and_validate_parameters()

        self._command_pub = self.create_publisher(Twist, "cmd_vel", COMMAND_QOS)
        self._status_pub = self.create_publisher(
            String, "command_selector/status", STATUS_QOS
        )
        self.create_subscription(
            Twist, "teleop/cmd_vel", self._on_teleop_command, COMMAND_QOS
        )
        self.create_subscription(
            Twist,
            "approach/cmd_vel_safe",
            self._on_approach_command,
            COMMAND_QOS,
        )
        self.create_subscription(
            Twist, "/nav2/cmd_vel", self._on_nav2_command, COMMAND_QOS
        )
        self.declare_parameter(
            "nav2_action_status_topic", "/navigate_to_pose/_action/status"
        )
        self._goal_selection_started = self.get_clock().now().nanoseconds
        self._seen_nav2_goal_ids = set()
        self._cancel_nav2_client = self.create_client(
            CancelGoal, "/navigate_to_pose/_action/cancel_goal"
        )
        self.create_subscription(
            String, "command_selector/request", self._on_source_request, COMMAND_QOS
        )
        self._seen_nav2_terminal_ids = set()
        self._active_nav2_goal_ids = set()
        self.create_subscription(
            GoalStatusArray,
            str(self.get_parameter("nav2_action_status_topic").value),
            self._on_nav2_action_status,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self._commands: Dict[CommandSource, Optional[PlanarCommand]] = {
            source: None for source in self._motion_sources()
        }
        self._received_seconds: Dict[CommandSource, Optional[float]] = {
            source: None for source in self._motion_sources()
        }
        self._last_status: Optional[str] = None
        self._parameter_callback = self.add_on_set_parameters_callback(
            self._on_parameter_change
        )
        self._timer = self.create_timer(1.0 / self._publish_rate, self._on_timer)
        self._publish_status(self._status_at(time.monotonic()))
        self.get_logger().info(
            "Command selector ready: source=%s, timeouts=(teleop %.3fs, "
            "approach %.3fs, nav2 %.3fs)"
            % (
                self._source.value,
                self._selector_parameters.teleop_timeout,
                self._selector_parameters.approach_timeout,
                self._selector_parameters.nav2_timeout,
            )
        )

    @staticmethod
    def _motion_sources():
        return (
            CommandSource.TELEOP,
            CommandSource.APPROACH,
            CommandSource.NAV2,
        )

    def _declare_parameters(self) -> None:
        """Declare startup configuration and runtime source mode."""
        self.declare_parameter("source_mode", CommandSource.STOP.value)
        self.declare_parameter("enable_nav2_goal_selection", False)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("teleop_timeout", 0.30)
        self.declare_parameter("approach_timeout", 0.35)
        self.declare_parameter("nav2_timeout", 0.50)
        self.declare_parameter("axis_epsilon", 1.0e-9)
        self.declare_parameter("shutdown_stop_count", 3)

    def _load_and_validate_parameters(self) -> None:
        """Load startup settings and reject ambiguous values."""
        source_value = str(self.get_parameter("source_mode").value)
        try:
            self._source = CommandSource(source_value)
        except ValueError as error:
            raise ValueError(
                "source_mode must be STOP, TELEOP, APPROACH, or NAV2"
            ) from error
        self._publish_rate = float(self.get_parameter("publish_rate").value)
        self._shutdown_stop_count = int(
            self.get_parameter("shutdown_stop_count").value
        )
        self._selector_parameters = SelectorParameters(
            teleop_timeout=float(self.get_parameter("teleop_timeout").value),
            approach_timeout=float(self.get_parameter("approach_timeout").value),
            nav2_timeout=float(self.get_parameter("nav2_timeout").value),
            axis_epsilon=float(self.get_parameter("axis_epsilon").value),
        )
        if not isfinite(self._publish_rate) or self._publish_rate <= 0.0:
            raise ValueError("publish_rate must be finite and greater than zero")
        if self._shutdown_stop_count < 1:
            raise ValueError("shutdown_stop_count must be at least one")
        self._selector_parameters.validate()

    def _on_teleop_command(self, message: Twist) -> None:
        self._on_command(CommandSource.TELEOP, message)

    def _on_approach_command(self, message: Twist) -> None:
        self._on_command(CommandSource.APPROACH, message)

    def _on_nav2_command(self, message: Twist) -> None:
        self._on_command(CommandSource.NAV2, message)

    def _on_nav2_action_status(self, message: GoalStatusArray) -> None:
        """Fail closed when a Nav2 goal ends, even if cmd_vel keeps streaming."""
        active_statuses = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        terminal_statuses = {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        }
        entries = [
            (bytes(entry.goal_info.goal_id.uuid), int(entry.status))
            for entry in message.status_list
        ]
        currently_active = {
            goal_id for goal_id, status in entries if status in active_statuses
        }
        new_goals = [
            entry for entry in message.status_list
            if int(entry.status) in active_statuses
            and bytes(entry.goal_info.goal_id.uuid) not in self._seen_nav2_goal_ids
            and (entry.goal_info.stamp.sec * 1_000_000_000
                 + entry.goal_info.stamp.nanosec) >= self._goal_selection_started
        ]
        self._seen_nav2_goal_ids.update(goal_id for goal_id, _ in entries)
        if new_goals and self.get_parameter("enable_nav2_goal_selection").value:
            self.set_parameters([Parameter("source_mode", value="NAV2")])
        self._active_nav2_goal_ids.update(currently_active)

        ended = [
            (goal_id, status)
            for goal_id, status in entries
            if status in terminal_statuses
        ]
        trigger = None
        if not currently_active:
            trigger = next(
                (
                    (goal_id, status)
                    for goal_id, status in ended
                    if goal_id in self._active_nav2_goal_ids
                    or goal_id not in self._seen_nav2_terminal_ids
                ),
                None,
            )
        self._seen_nav2_terminal_ids.update(goal_id for goal_id, _ in ended)
        self._active_nav2_goal_ids.difference_update(goal_id for goal_id, _ in ended)

        if self._source != CommandSource.NAV2 or trigger is None:
            return

        _terminal_id, terminal_status = trigger
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }.get(terminal_status, str(terminal_status))
        self.get_logger().error(
            "Nav2 goal ended (%s); switching command source to STOP" % status_name
        )
        result = self.set_parameters([Parameter("source_mode", value="STOP")])[0]
        if not result.successful:
            self._source = CommandSource.STOP
            self._clear_commands()
            self._command_pub.publish(Twist())
            self._publish_status("STOP")
            self.get_logger().error(
                "Could not update source_mode parameter; forced internal STOP"
            )

    def _on_source_request(self, message: String) -> None:
        if message.data not in {"TELEOP", "STOP"}:
            return
        was_nav2 = self._source == CommandSource.NAV2
        self.set_parameters([Parameter("source_mode", value=message.data)])
        if was_nav2 and self._cancel_nav2_client.service_is_ready():
            future = self._cancel_nav2_client.call_async(CancelGoal.Request())
            future.add_done_callback(self._on_cancel_result)

    def _on_cancel_result(self, future) -> None:
        try:
            result = future.result()
            self.get_logger().info(
                f"Keyboard takeover: Nav2 cancellation code {result.return_code}"
            )
        except Exception as exc:
            self.get_logger().error(f"Nav2 cancellation failed: {exc}")

    def _on_command(self, source: CommandSource, message: Twist) -> None:
        """Cache only a valid command from the currently selected source."""
        if self._source != source:
            return
        command = sanitize_command(
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
            self._selector_parameters.axis_epsilon,
        )
        self._commands[source] = command
        self._received_seconds[source] = (
            time.monotonic() if command is not None else None
        )
        if command is None:
            self._command_pub.publish(Twist())
            self._publish_status(f"INVALID_{source.value}")
            self.get_logger().warning(
                "Rejected invalid %s velocity command" % source.value,
                throttle_duration_sec=1.0,
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
                reason="source_mode must be STOP, TELEOP, APPROACH, or NAV2",
            )

        if source != self._source:
            self._source = source
            self._clear_commands()
            self._command_pub.publish(Twist())
            self._publish_status(self._status_at(time.monotonic()))
            self.get_logger().info(
                "Command source changed to %s; output zero until a fresh "
                "selected-source command arrives" % source.value
            )
        return SetParametersResult(successful=True)

    def _on_timer(self) -> None:
        """Publish the selected fresh command or an explicit zero."""
        now_seconds = time.monotonic()
        command, received_seconds = self._selected_cache()
        selected = select_command(
            self._source,
            now_seconds,
            self._selector_parameters,
            command,
            received_seconds,
        )
        self._command_pub.publish(self._to_twist(selected))
        self._publish_status(
            selection_status(
                self._source,
                now_seconds,
                self._selector_parameters,
                command,
                received_seconds,
            )
        )

    def _selected_cache(self):
        if self._source == CommandSource.STOP:
            return None, None
        return (
            self._commands[self._source],
            self._received_seconds[self._source],
        )

    def _status_at(self, now_seconds: float) -> str:
        command, received_seconds = self._selected_cache()
        return selection_status(
            self._source,
            now_seconds,
            self._selector_parameters,
            command,
            received_seconds,
        )

    def _publish_status(self, status: str) -> None:
        if status == self._last_status:
            return
        self._last_status = status
        self._status_pub.publish(String(data=status))
        if status.startswith("STALE_"):
            self.get_logger().warning("Command selector status: %s" % status)
        else:
            self.get_logger().info("Command selector status: %s" % status)

    def _clear_commands(self) -> None:
        for source in self._motion_sources():
            self._commands[source] = None
            self._received_seconds[source] = None

    @staticmethod
    def _to_twist(command: PlanarCommand) -> Twist:
        message = Twist()
        message.linear.x = command.linear_x
        message.angular.z = command.angular_z
        return message

    def stop(self) -> None:
        """Publish an explicit zero burst before process shutdown."""
        stop = Twist()
        for _ in range(self._shutdown_stop_count):
            self._command_pub.publish(stop)


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
