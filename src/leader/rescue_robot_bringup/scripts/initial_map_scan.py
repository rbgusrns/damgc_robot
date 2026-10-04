#!/usr/bin/env python3
"""Perform one stationary 360-degree Nav2 spin before mapping exploration."""

import math
import time
from typing import Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from nav2_msgs.action import Spin
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParameters
from std_srvs.srv import Trigger


class InitialMapScan(Node):
    """Wait for a stable pose, scan once, then return command ownership to STOP."""

    def __init__(self) -> None:
        super().__init__("initial_map_scan")
        self.declare_parameter("enabled", False)
        self.declare_parameter("odometry_topic", "/visual_slam/tracking/odometry")
        self.declare_parameter("selector_node", "/leader/command_selector")
        self.declare_parameter("spin_action", "/spin")
        self.declare_parameter("target_yaw", 2.0 * math.pi)
        self.declare_parameter("startup_timeout", 120.0)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.target_yaw = float(self.get_parameter("target_yaw").value)
        self.startup_timeout = float(self.get_parameter("startup_timeout").value)
        self._started_at = time.monotonic()
        self._done = not self.enabled
        self._triggered = not self.enabled
        self._state = "WAITING"
        self._stable_since: Optional[float] = None
        self._last_pose: Optional[Tuple[float, float, float]] = None
        self._last_odom_time: Optional[float] = None
        self._last_feedback_log = 0.0
        self._owns_nav2_mode = False

        odometry_topic = str(self.get_parameter("odometry_topic").value)
        selector_node = str(self.get_parameter("selector_node").value)
        spin_action = str(self.get_parameter("spin_action").value)
        self._get_selector_parameters = self.create_client(
            GetParameters, f"{selector_node}/get_parameters"
        )
        self._set_selector_parameters = self.create_client(
            SetParameters, f"{selector_node}/set_parameters"
        )
        self._spin = ActionClient(self, Spin, spin_action)
        self._odom_sub = self.create_subscription(
            Odometry, odometry_topic, self._on_odometry, 10
        )
        self._start_service = self.create_service(
            Trigger, "/leader/initial_map_scan/start", self._on_start_request
        )
        self._timer = self.create_timer(0.2, self._tick)

        if self.enabled:
            self.get_logger().info(
                "Initial mapping scan enabled: waiting for the rosbag trigger, "
                "stable odometry, and Nav2 Spin."
            )
        else:
            self.get_logger().info("Initial mapping scan disabled.")

    @staticmethod
    def _yaw(orientation) -> float:
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def _angle_delta(a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def _on_odometry(self, message: Odometry) -> None:
        now = time.monotonic()
        pose = message.pose.pose
        current = (pose.position.x, pose.position.y, self._yaw(pose.orientation))
        if self._last_pose is None or self._last_odom_time is None:
            self._stable_since = now
        else:
            dt = now - self._last_odom_time
            distance = math.hypot(current[0] - self._last_pose[0], current[1] - self._last_pose[1])
            yaw_delta = abs(self._angle_delta(current[2], self._last_pose[2]))
            if dt > 0.5 or distance > 0.025 or yaw_delta > 0.06:
                self._stable_since = now
            elif self._stable_since is None:
                self._stable_since = now
        self._last_pose = current
        self._last_odom_time = now

    def _on_start_request(self, request, response):
        del request
        if not self.enabled:
            response.success = False
            response.message = "Initial scan is disabled."
        elif self._triggered:
            response.success = False
            response.message = "Initial scan was already requested."
        else:
            self._triggered = True
            response.success = True
            response.message = "Initial scan queued; waiting for prerequisites."
            self.get_logger().info(response.message)
        return response

    def _tick(self) -> None:
        if self._done:
            return
        now = time.monotonic()
        if now - self._started_at > self.startup_timeout:
            self.get_logger().error("Timed out waiting for the initial scan prerequisites.")
            self._finish()
            return
        if not self._triggered:
            return

        odometry_fresh = (
            self._last_odom_time is not None
            and now - self._last_odom_time < 0.5
            and self._stable_since is not None
            and now - self._stable_since >= 1.0
        )
        if not odometry_fresh or not self._spin.server_is_ready():
            return

        if self._state == "WAITING":
            if not self._get_selector_parameters.service_is_ready():
                return
            self._state = "CHECKING_SELECTOR"
            request = GetParameters.Request()
            request.names = ["source_mode"]
            future = self._get_selector_parameters.call_async(request)
            future.add_done_callback(self._on_selector_read)

    def _on_selector_read(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - ROS service failures are runtime input.
            self.get_logger().error(f"Could not read command selector mode: {error}")
            self._finish()
            return
        values = response.values
        mode = values[0].string_value if values else "unknown"
        if not values or mode != "STOP":
            self.get_logger().error(
                f"Initial scan requires selector STOP; current mode is {mode}."
            )
            self._finish()
            return

        self._state = "SETTING_NAV2"
        future = self._set_selector_mode("NAV2")
        future.add_done_callback(self._on_nav2_selected)

    def _on_nav2_selected(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - ROS service failures are runtime input.
            self.get_logger().error(f"Could not enable Nav2 command source: {error}")
            self._finish()
            return
        results = response.results
        if not results or not results[0].successful:
            reason = results[0].reason if results else "no selector response"
            self.get_logger().error(f"Could not enable Nav2 command source: {reason}")
            self._finish()
            return

        self._owns_nav2_mode = True
        goal = Spin.Goal()
        goal.target_yaw = self.target_yaw
        goal.time_allowance = Duration(sec=65, nanosec=0)
        self._state = "SPINNING"
        self.get_logger().info(
            f"Starting initial {math.degrees(self.target_yaw):.0f}-degree mapping scan."
        )
        send_future = self._spin.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001 - ROS action failures are runtime input.
            self.get_logger().error(f"Could not start the initial scan: {error}")
            self._finish()
            return
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected the initial scan goal.")
            self._finish()
            return
        self.get_logger().info("Nav2 accepted the initial scan goal.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_message) -> None:
        now = time.monotonic()
        if now - self._last_feedback_log < 2.0:
            return
        self._last_feedback_log = now
        angle = feedback_message.feedback.angular_distance_traveled
        self.get_logger().info(f"Initial scan progress: {math.degrees(angle):.0f} degrees.")

    def _on_result(self, future) -> None:
        try:
            wrapped_result = future.result()
        except Exception as error:  # noqa: BLE001 - ROS action failures are runtime input.
            self.get_logger().error(f"Initial scan result failed: {error}")
            self._finish()
            return
        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Initial 360-degree map scan completed.")
        else:
            self.get_logger().error(
                f"Initial map scan ended with Nav2 status {wrapped_result.status}."
            )
        self._finish()

    def _finish(self) -> None:
        if self._done:
            return
        self._done = True
        if not self._owns_nav2_mode or not self._set_selector_parameters.service_is_ready():
            return
        future = self._set_selector_mode("STOP")
        future.add_done_callback(self._on_stopped)

    def _on_stopped(self, future) -> None:
        try:
            response = future.result()
            results = response.results
            if results and results[0].successful:
                self.get_logger().info("Command selector returned to STOP after initial scan.")
            else:
                self.get_logger().error("Could not return command selector to STOP.")
        except Exception as error:  # noqa: BLE001 - ROS service failures are runtime input.
            self.get_logger().error(f"Could not return command selector to STOP: {error}")

    def _set_selector_mode(self, mode: str):
        request = SetParameters.Request()
        request.parameters = [
            Parameter("source_mode", value=mode).to_parameter_msg()
        ]
        return self._set_selector_parameters.call_async(request)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialMapScan()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.2)
        # Give the asynchronous STOP request a chance to complete after the
        # action result callback has marked the scan complete.
        if node._owns_nav2_mode and node._set_selector_parameters.service_is_ready():
            future = node._set_selector_mode("STOP")
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted; returning command selector to STOP.")
        if node._owns_nav2_mode and node._set_selector_parameters.service_is_ready():
            future = node._set_selector_mode("STOP")
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
