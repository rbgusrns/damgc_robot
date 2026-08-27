#!/usr/bin/env python3
"""Publish Leader approach state from camera-to-AprilTag transforms.

The configured source frame must be a camera optical frame, where x points
right, y points down, and z points forward.  The node publishes observations in
that frame only; it does not command motion or transform data into ``base_link``.
"""

from math import isfinite
from typing import List, Optional, Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float64, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener

from rescue_robot_apriltag.approach_logic import (
    ApproachState,
    ApproachStateMachine,
    ApproachThresholds,
    MedianTranslationFilter,
    RelativeMeasurement,
    TagObservation,
    is_valid_translation,
    normalize_quaternion,
    select_observation,
)


class AprilTagApproachNode(Node):
    """Select fresh Leader tag TFs and publish relative alignment state."""

    def __init__(self) -> None:
        super().__init__("apriltag_approach")
        self._declare_parameters()
        self._load_and_validate_parameters()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Relative topic names resolve below /leader when the node is namespaced.
        self._detected_pub = self.create_publisher(Bool, "supply/detected", 10)
        self._tag_id_pub = self.create_publisher(Int32, "supply/tag_id", 10)
        self._pose_pub = self.create_publisher(
            PoseStamped, "supply/relative_pose", 10
        )
        self._distance_pub = self.create_publisher(Float64, "supply/distance", 10)
        self._lateral_pub = self.create_publisher(
            Float64, "supply/lateral_error", 10
        )
        self._straight_pub = self.create_publisher(
            Float64, "supply/straight_distance", 10
        )
        self._angle_pub = self.create_publisher(Float64, "supply/angle", 10)
        self._state_pub = self.create_publisher(String, "alignment/state", 10)

        self._translation_filter = MedianTranslationFilter(self._filter_window)
        self._state_machine = ApproachStateMachine(
            ApproachThresholds(
                target_distance=self._target_distance,
                distance_tolerance=self._distance_tolerance,
                lateral_tolerance=self._lateral_tolerance,
                angle_tolerance_deg=self._angle_tolerance_deg,
                stable_time=self._stable_time,
            )
        )
        self._active_tag_id: Optional[int] = None
        self._last_logged_state: Optional[ApproachState] = None
        self._timer = self.create_timer(1.0 / self._publish_rate, self._on_timer)

    def _declare_parameters(self) -> None:
        """Declare startup-only Leader approach parameters."""
        self.declare_parameter("source_frame", "camera_color_optical_frame")
        self.declare_parameter("tag_frame_pattern", "leader/tag36h11:{id}")
        self.declare_parameter("target_tag_id", 0)
        self.declare_parameter("allowed_tag_ids", [0, 1, 2])
        self.declare_parameter("selection_mode", "priority")
        self.declare_parameter("target_distance", 0.15)
        self.declare_parameter("distance_tolerance", 0.02)
        self.declare_parameter("lateral_tolerance", 0.02)
        self.declare_parameter("angle_tolerance_deg", 5.0)
        self.declare_parameter("tag_timeout", 1.0)
        self.declare_parameter("stable_time", 0.8)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("filter_window", 5)

    def _load_and_validate_parameters(self) -> None:
        """Load parameters and reject ambiguous or unsafe configurations."""
        self._source_frame = str(self.get_parameter("source_frame").value)
        self._tag_frame_pattern = str(
            self.get_parameter("tag_frame_pattern").value
        )
        self._target_tag_id = int(self.get_parameter("target_tag_id").value)
        self._allowed_tag_ids = [
            int(tag_id) for tag_id in self.get_parameter("allowed_tag_ids").value
        ]
        self._selection_mode = str(self.get_parameter("selection_mode").value)
        self._target_distance = float(self.get_parameter("target_distance").value)
        self._distance_tolerance = float(
            self.get_parameter("distance_tolerance").value
        )
        self._lateral_tolerance = float(
            self.get_parameter("lateral_tolerance").value
        )
        self._angle_tolerance_deg = float(
            self.get_parameter("angle_tolerance_deg").value
        )
        self._tag_timeout = float(self.get_parameter("tag_timeout").value)
        self._stable_time = float(self.get_parameter("stable_time").value)
        self._publish_rate = float(self.get_parameter("publish_rate").value)
        self._filter_window = int(self.get_parameter("filter_window").value)

        if not self._source_frame:
            raise ValueError("source_frame must not be empty")
        if "{id}" not in self._tag_frame_pattern:
            raise ValueError("tag_frame_pattern must contain the '{id}' placeholder")
        try:
            self._tag_frame_pattern.format(id=0)
        except (KeyError, IndexError, ValueError) as error:
            raise ValueError("tag_frame_pattern is not a valid format string") from error
        if self._target_tag_id < -1:
            raise ValueError("target_tag_id must be -1 or a non-negative integer")
        if any(tag_id < 0 for tag_id in self._allowed_tag_ids):
            raise ValueError("allowed_tag_ids must contain only non-negative IDs")
        if len(set(self._allowed_tag_ids)) != len(self._allowed_tag_ids):
            raise ValueError("allowed_tag_ids must not contain duplicates")
        if self._target_tag_id == -1 and not self._allowed_tag_ids:
            raise ValueError("allowed_tag_ids must not be empty in multi-tag mode")
        if self._selection_mode not in ("priority", "nearest"):
            raise ValueError("selection_mode must be 'priority' or 'nearest'")

        numeric_values = (
            self._target_distance,
            self._distance_tolerance,
            self._lateral_tolerance,
            self._angle_tolerance_deg,
            self._tag_timeout,
            self._stable_time,
            self._publish_rate,
        )
        if not all(isfinite(value) for value in numeric_values):
            raise ValueError("Numeric parameters must be finite")
        ApproachThresholds(
            target_distance=self._target_distance,
            distance_tolerance=self._distance_tolerance,
            lateral_tolerance=self._lateral_tolerance,
            angle_tolerance_deg=self._angle_tolerance_deg,
            stable_time=self._stable_time,
        ).validate()
        if self._tag_timeout < 0.0:
            raise ValueError("tag_timeout must not be negative")
        if self._publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")
        if self._filter_window < 1:
            raise ValueError("filter_window must be at least 1")

    def _candidate_ids(self) -> Sequence[int]:
        """Return the tag IDs eligible for the current lookup cycle."""
        if self._target_tag_id >= 0:
            return (self._target_tag_id,)
        return self._allowed_tag_ids

    def _on_timer(self) -> None:
        """Look up candidates, select and filter one, then publish one cycle."""
        now = self.get_clock().now()
        candidate_ids = self._candidate_ids()
        observations = [
            observation
            for tag_id in candidate_ids
            if (observation := self._lookup_observation(tag_id, now)) is not None
        ]
        selected = select_observation(
            observations, list(candidate_ids), self._selection_mode
        )
        if selected is None:
            self._publish_lost(now.nanoseconds / 1.0e9)
            return

        if selected.tag_id != self._active_tag_id:
            self._translation_filter.reset()
            self._state_machine.reset()
            self._active_tag_id = selected.tag_id

        measurement = self._translation_filter.add(
            selected.x,
            selected.y,
            selected.z,
            selected.stamp_nanoseconds,
        )
        state = self._state_machine.update(
            measurement, now.nanoseconds / 1.0e9, selected.tag_id
        )
        self._publish_valid(selected, measurement, state)

    def _lookup_observation(self, tag_id: int, now: Time) -> Optional[TagObservation]:
        """Return one fresh, valid tag transform or ``None`` when unavailable."""
        tag_frame = self._tag_frame_pattern.format(id=tag_id)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._source_frame, tag_frame, Time()
            )
        except TransformException:
            # Missing transforms are normal while a tag is outside the D435 view.
            return None

        stamp = Time.from_msg(transform.header.stamp)
        age_seconds = (now.nanoseconds - stamp.nanoseconds) / 1.0e9
        if age_seconds > self._tag_timeout:
            return None

        translation = transform.transform.translation
        if not is_valid_translation(translation.x, translation.y, translation.z):
            return None
        rotation = transform.transform.rotation
        quaternion = normalize_quaternion(
            (rotation.x, rotation.y, rotation.z, rotation.w)
        )
        if quaternion is None:
            return None

        return TagObservation(
            tag_id=tag_id,
            x=float(translation.x),
            y=float(translation.y),
            z=float(translation.z),
            quaternion=quaternion,
            stamp_nanoseconds=stamp.nanoseconds,
        )

    def _publish_lost(self, now_seconds: float) -> None:
        """Publish only loss outputs and clear all temporal measurement state."""
        self._translation_filter.reset()
        self._active_tag_id = None
        state = self._state_machine.update(None, now_seconds, None)
        self._detected_pub.publish(Bool(data=False))
        self._tag_id_pub.publish(Int32(data=-1))
        self._state_pub.publish(String(data=state.value))
        self._log_state_change(state)

    def _publish_valid(
        self,
        selected: TagObservation,
        measurement: RelativeMeasurement,
        state: ApproachState,
    ) -> None:
        """Publish a fresh pose, metrics, selected identity, and approach state."""
        pose = PoseStamped()
        pose.header.frame_id = self._source_frame
        pose.header.stamp = Time(nanoseconds=selected.stamp_nanoseconds).to_msg()
        pose.pose.position.x = measurement.x
        pose.pose.position.y = measurement.y
        pose.pose.position.z = measurement.z
        pose.pose.orientation.x = selected.quaternion[0]
        pose.pose.orientation.y = selected.quaternion[1]
        pose.pose.orientation.z = selected.quaternion[2]
        pose.pose.orientation.w = selected.quaternion[3]

        self._detected_pub.publish(Bool(data=True))
        self._tag_id_pub.publish(Int32(data=selected.tag_id))
        self._pose_pub.publish(pose)
        self._distance_pub.publish(Float64(data=measurement.distance))
        self._lateral_pub.publish(Float64(data=measurement.lateral_error))
        self._straight_pub.publish(Float64(data=measurement.straight_distance))
        self._angle_pub.publish(Float64(data=measurement.angle))
        self._state_pub.publish(String(data=state.value))
        self._log_state_change(state)

    def _log_state_change(self, state: ApproachState) -> None:
        """Log state transitions once instead of logging at timer frequency."""
        if state != self._last_logged_state:
            self.get_logger().info("Alignment state changed to %s" % state.value)
            self._last_logged_state = state


def main(args: Optional[List[str]] = None) -> None:
    """Initialize ROS 2 and spin the Leader AprilTag approach node."""
    rclpy.init(args=args)
    node: Optional[AprilTagApproachNode] = None
    try:
        node = AprilTagApproachNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


if __name__ == "__main__":
    main()
