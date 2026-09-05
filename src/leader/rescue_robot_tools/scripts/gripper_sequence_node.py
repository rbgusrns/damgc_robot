#!/usr/bin/env python3
"""Run a guarded AprilTag-to-gripper sequence.

This node does not replace the AprilTag node or the Dynamixel driver.
It only connects their existing ROS interfaces:

  /leader/apriltag_approach/alignment/state -> /leader/gripper/command

When the alignment state stays ALIGNED, the node sends:
  open -> close -> rx64_middle

The default values are deliberately slow and conservative for bench testing.
The sequence runs once per continuous alignment event.  It can run again only
after the alignment state leaves ALIGNED (for example TAG_LOST).
"""

from __future__ import annotations

from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SequenceState(str, Enum):
    WAITING = "WAITING"
    OPENING = "OPENING"
    CLOSING = "CLOSING"
    LIFTING = "LIFTING"
    DONE = "DONE"


class GripperSequenceNode(Node):
    """Convert a stable AprilTag alignment into gripper commands."""

    def __init__(self) -> None:
        super().__init__("gripper_sequence")

        self.declare_parameter(
            "alignment_topic", "/leader/apriltag_approach/alignment/state"
        )
        self.declare_parameter("gripper_topic", "/leader/gripper/command")
        self.declare_parameter("open_wait", 2.0)
        self.declare_parameter("close_wait", 2.0)
        self.declare_parameter("lift_wait", 2.0)
        self.declare_parameter("require_aligned_repeats", 3)
        self.declare_parameter("enabled", False)

        alignment_topic = str(self.get_parameter("alignment_topic").value)
        gripper_topic = str(self.get_parameter("gripper_topic").value)
        self._open_wait = float(self.get_parameter("open_wait").value)
        self._close_wait = float(self.get_parameter("close_wait").value)
        self._lift_wait = float(self.get_parameter("lift_wait").value)
        self._required_repeats = int(
            self.get_parameter("require_aligned_repeats").value
        )
        self._enabled = bool(self.get_parameter("enabled").value)

        if self._open_wait < 0 or self._close_wait < 0 or self._lift_wait < 0:
            raise ValueError("wait parameters must be non-negative")
        if self._required_repeats < 1:
            raise ValueError("require_aligned_repeats must be at least 1")

        self._command_pub = self.create_publisher(String, gripper_topic, 10)
        self._status_pub = self.create_publisher(String, "sequence/status", 10)
        self._state_sub = self.create_subscription(
            String, alignment_topic, self._alignment_callback, 10
        )
        self._timer = self.create_timer(0.05, self._on_timer)

        self._sequence_state = SequenceState.WAITING
        self._last_alignment = "TAG_LOST"
        self._aligned_repeats = 0
        self._deadline: Optional[float] = None
        self._has_run_for_alignment = False
        self._publish_status("WAITING enabled=%s" % self._enabled)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def _publish_command(self, command: str) -> None:
        message = String(data=command)
        self._command_pub.publish(message)
        self.get_logger().info("gripper command: %s" % command)

    def _publish_status(self, status: str) -> None:
        self._status_pub.publish(String(data=status))
        self.get_logger().info(status)

    def _alignment_callback(self, message: String) -> None:
        state = message.data.strip().upper()
        self._last_alignment = state

        if state == "ALIGNED":
            self._aligned_repeats += 1
        else:
            self._aligned_repeats = 0
            # A lost/non-aligned tag arms the node for the next alignment.
            if self._sequence_state == SequenceState.DONE:
                self._sequence_state = SequenceState.WAITING
                self._has_run_for_alignment = False
                self._publish_status("WAITING next alignment armed")

    def _on_timer(self) -> None:
        if not self._enabled:
            return

        now = self._now()
        if self._sequence_state == SequenceState.WAITING:
            if (
                self._last_alignment == "ALIGNED"
                and self._aligned_repeats >= self._required_repeats
                and not self._has_run_for_alignment
            ):
                self._has_run_for_alignment = True
                self._sequence_state = SequenceState.OPENING
                self._publish_command("open")
                self._deadline = now + self._open_wait
                self._publish_status("OPENING")
            return

        if self._deadline is None or now < self._deadline:
            return

        if self._sequence_state == SequenceState.OPENING:
            self._sequence_state = SequenceState.CLOSING
            self._publish_command("close")
            self._deadline = now + self._close_wait
            self._publish_status("CLOSING")
        elif self._sequence_state == SequenceState.CLOSING:
            self._sequence_state = SequenceState.LIFTING
            self._publish_command("rx64_middle")
            self._deadline = now + self._lift_wait
            self._publish_status("LIFTING")
        elif self._sequence_state == SequenceState.LIFTING:
            self._sequence_state = SequenceState.DONE
            self._deadline = None
            self._publish_status("DONE")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperSequenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
