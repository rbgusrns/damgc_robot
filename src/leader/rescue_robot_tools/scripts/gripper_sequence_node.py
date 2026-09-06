#!/usr/bin/env python3
"""Open on AprilTag detection and close when alignment becomes ALIGNED."""

from __future__ import annotations

import math
from enum import Enum
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String

from dynamixel_orin import get_profile


class SequenceState(str, Enum):
    WAITING_FOR_TAG = "WAITING_FOR_TAG"
    OPENING = "OPENING"
    CLOSING = "CLOSING"
    LIFTING = "LIFTING"
    DONE = "DONE"


class GripperSequenceNode(Node):
    """Connect the existing AprilTag state outputs to Dynamixel commands."""

    def __init__(self) -> None:
        super().__init__("gripper_sequence")
        self.declare_parameter("detection_topic", "/leader/supply/detected")
        self.declare_parameter("alignment_topic", "/leader/base_alignment/state")
        self.declare_parameter("raw_command_topic", "/leader/dynamixel/command")
        self.declare_parameter("gripper_topic", "/leader/gripper/command")
        self.declare_parameter("open_raw", 950)
        self.declare_parameter("close_raw", 350)
        self.declare_parameter("close_wait", 3.0)
        self.declare_parameter("enabled", False)
        self.declare_parameter("robot", "leader")
        self.declare_parameter("lift_enabled", False)
        self.declare_parameter("lift_raw", 350.0)

        detection_topic = str(self.get_parameter("detection_topic").value)
        alignment_topic = str(self.get_parameter("alignment_topic").value)
        raw_topic = str(self.get_parameter("raw_command_topic").value)
        gripper_topic = str(self.get_parameter("gripper_topic").value)
        self._open_raw = int(self.get_parameter("open_raw").value)
        self._close_raw = int(self.get_parameter("close_raw").value)
        self._close_wait = float(self.get_parameter("close_wait").value)
        self._enabled = bool(self.get_parameter("enabled").value)
        self._lift_enabled = bool(self.get_parameter("lift_enabled").value)
        # RX-64 remains disabled by default until its mechanical direction is validated.
        lift_raw_value = self.get_parameter("lift_raw").value
        try:
            self._lift_raw_value = float(lift_raw_value)
        except (TypeError, ValueError):
            self._lift_raw_value = math.nan
        self._lift_raw = (
            int(self._lift_raw_value)
            if math.isfinite(self._lift_raw_value)
            and self._lift_raw_value.is_integer()
            else -1
        )
        robot = str(self.get_parameter("robot").value)
        profile = get_profile(robot)
        self._lift_min_raw = int(profile["rx64_min"])
        self._lift_max_raw = int(profile["rx64_max"])
        self._lift_raw_valid = (
            math.isfinite(self._lift_raw_value)
            and self._lift_raw_value.is_integer()
            and self._lift_min_raw <= self._lift_raw <= self._lift_max_raw
        )

        if not 1 <= self._open_raw <= 1021:
            raise ValueError("open_raw must be between 1 and 1021")
        if not 1 <= self._close_raw <= 1021:
            raise ValueError("close_raw must be between 1 and 1021")

        self._raw_pub = self.create_publisher(Float64MultiArray, raw_topic, 10)
        self._gripper_pub = self.create_publisher(String, gripper_topic, 10)
        self._status_pub = self.create_publisher(String, "sequence/status", 10)
        self.create_subscription(Bool, detection_topic, self._detection_callback, 10)
        self.create_subscription(String, alignment_topic, self._alignment_callback, 10)
        self._timer = self.create_timer(0.05, self._on_timer)

        self._state = SequenceState.WAITING_FOR_TAG
        self._tag_detected = False
        self._alignment_state = "TAG_LOST"
        self._deadline: Optional[float] = None
        self._publish_status(
            "WAITING_FOR_TAG enabled=%s detection=%s alignment=%s open_raw=%d "
            "close_raw=%d lift_enabled=%s lift_raw=%s"
            % (
                self._enabled,
                detection_topic,
                alignment_topic,
                self._open_raw,
                self._close_raw,
                self._lift_enabled,
                lift_raw_value,
            )
        )
        if self._lift_enabled and not self._lift_raw_valid:
            self._publish_status(
                "ERROR invalid lift_raw=%s; expected integer %d..%d; RX-64 lift disabled"
                % (lift_raw_value, self._lift_min_raw, self._lift_max_raw)
            )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def _publish_status(self, text: str) -> None:
        self._status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _publish_raw(
        self,
        rx64: float,
        rx28: float,
        torque: float,
        rx64_torque: Optional[float] = None,
        rx28_torque: Optional[float] = None,
    ) -> None:
        data = [rx64, rx28, torque]
        if rx64_torque is not None and rx28_torque is not None:
            data.extend([rx64_torque, rx28_torque])
        self._raw_pub.publish(Float64MultiArray(data=[rx64, rx28, torque]))
        self.get_logger().info(
            "raw command rx64=%s rx28=%s torque=%s" % (rx64, rx28, torque)
        )

    def _publish_gripper(self, command: str) -> None:
        self._gripper_pub.publish(String(data=command))
        self.get_logger().info("gripper command: %s" % command)

    def _detection_callback(self, message: Bool) -> None:
        if not self._enabled:
            return
        detected = bool(message.data)
        self._tag_detected = detected
        if detected and self._state == SequenceState.WAITING_FOR_TAG:
            self._state = SequenceState.OPENING
            self._publish_raw(-1.0, float(self._open_raw), -1.0, -1.0, 1.0)
            self._publish_status("OPENING tag_detected")

    def _alignment_callback(self, message: String) -> None:
        self._alignment_state = message.data.strip().upper()

    def _on_timer(self) -> None:
        if not self._enabled:
            return

        if self._state == SequenceState.OPENING:
            if self._alignment_state == "ALIGNED":
                self._state = SequenceState.CLOSING
                self._publish_raw(-1.0, float(self._close_raw), -1.0, -1.0, 1.0)
                self._publish_status("CLOSING alignment=ALIGNED close_raw=%d" % self._close_raw)
                if self._lift_enabled and self._lift_raw_valid:
                    self._deadline = self._now() + self._close_wait
                else:
                    self._state = SequenceState.DONE
                    self._deadline = None
                    if self._lift_enabled:
                        self._publish_status("DONE lift_skipped_invalid_raw")
                    else:
                        self._publish_status("DONE lift_disabled")
                    return
            return

        if self._state == SequenceState.CLOSING:
            if self._deadline is not None and self._now() >= self._deadline:
                self._state = SequenceState.LIFTING
                self._publish_raw(float(self._lift_raw), -1.0, -1.0, 1.0, -1.0)
                self._state = SequenceState.DONE
                self._deadline = None
                self._publish_status("DONE lift_raw=%d" % self._lift_raw)
            return

        if self._state == SequenceState.LIFTING:
            if self._deadline is not None and self._now() >= self._deadline:
                self._state = SequenceState.DONE
                self._deadline = None
                self._publish_status("DONE")
            return

        if (
            self._state == SequenceState.DONE
            and not self._tag_detected
            and self._alignment_state != "ALIGNED"
        ):
            self._state = SequenceState.WAITING_FOR_TAG
            self._publish_status("WAITING_FOR_TAG next cycle armed")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperSequenceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
