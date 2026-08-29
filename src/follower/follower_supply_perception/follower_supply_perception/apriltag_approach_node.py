"""ROS 2 node that publishes AprilTag-relative approach and alignment state."""

from math import isfinite
from typing import List, Optional, Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float64, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener

from follower_supply_perception.approach_logic import (
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
from follower_supply_perception.base_alignment_logic import (
    BaseAlignmentMeasurement,
    BaseAlignmentStateMachine,
    BaseAlignmentThresholds,
)
from follower_supply_perception.base_pose import (
    compute_base_metrics,
    is_fresh_timestamp,
    transform_pose_preserving_stamp,
)


class AprilTagApproachNode(Node):
    """Look up AprilTag TFs and publish filtered relative approach state."""

    def __init__(self) -> None:
        super().__init__("apriltag_approach")
        self._declare_parameters()
        self._load_and_validate_parameters()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Relative names become /follower/... when launched in the follower namespace.
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
        self._base_pose_pub = self.create_publisher(
            PoseStamped, "supply/base_relative_pose", 10
        )
        self._base_forward_pub = self.create_publisher(
            Float64, "supply/base_forward_distance", 10
        )
        self._base_lateral_pub = self.create_publisher(
            Float64, "supply/base_lateral_error", 10
        )
        self._base_bearing_pub = self.create_publisher(
            Float64, "supply/base_bearing", 10
        )
        self._base_state_pub = self.create_publisher(
            String, "base_alignment/state", 10
        )

        self._translation_filter = MedianTranslationFilter(self._filter_window)
        thresholds = ApproachThresholds(
            target_distance=self._target_distance,
            distance_tolerance=self._distance_tolerance,
            lateral_tolerance=self._lateral_tolerance,
            angle_tolerance_deg=self._angle_tolerance_deg,
            stable_time=self._stable_time,
        )
        self._state_machine = ApproachStateMachine(thresholds)
        self._base_state_machine = BaseAlignmentStateMachine(
            BaseAlignmentThresholds(
                target_forward=self._base_target_forward,
                forward_tolerance=self._base_forward_tolerance,
                lateral_tolerance=self._base_lateral_tolerance,
                bearing_tolerance_deg=self._base_bearing_tolerance_deg,
                stable_time=self._base_stable_time,
                sample_timeout=self._tag_timeout,
            )
        )
        self._active_tag_id: Optional[int] = None
        self._last_logged_state: Optional[ApproachState] = None
        self._last_logged_base_state: Optional[ApproachState] = None
        self._timer = self.create_timer(1.0 / self._publish_rate, self._on_timer)

    def _declare_parameters(self) -> None:
        """Declare all startup-only configuration parameters."""
        self.declare_parameter(
            "source_frame", "follower/follower_camera_optical_frame"
        )
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tf_lookup_timeout", 0.05)
        self.declare_parameter("tag_frame_pattern", "follower/tag36h11:{id}")
        self.declare_parameter("target_tag_id", 0)
        self.declare_parameter("allowed_tag_ids", [0, 1, 2])
        self.declare_parameter("selection_mode", "priority")
        self.declare_parameter("target_distance", 0.15)
        self.declare_parameter("distance_tolerance", 0.02)
        self.declare_parameter("lateral_tolerance", 0.02)
        self.declare_parameter("angle_tolerance_deg", 5.0)
        self.declare_parameter("tag_timeout", 0.3)
        self.declare_parameter("stable_time", 0.8)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("filter_window", 5)
        # Match the Leader's configured base-frame approach stop target.
        self.declare_parameter("base_target_forward", 0.25)
        self.declare_parameter("base_forward_tolerance", 0.03)
        self.declare_parameter("base_lateral_tolerance", 0.02)
        self.declare_parameter("base_bearing_tolerance_deg", 5.0)
        self.declare_parameter("base_stable_time", 0.8)

    def _load_and_validate_parameters(self) -> None:
        """Load parameters once and reject ambiguous or unsafe configurations."""
        self._source_frame = str(self.get_parameter("source_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tf_lookup_timeout = float(
            self.get_parameter("tf_lookup_timeout").value
        )
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
        self._base_target_forward = float(
            self.get_parameter("base_target_forward").value
        )
        self._base_forward_tolerance = float(
            self.get_parameter("base_forward_tolerance").value
        )
        self._base_lateral_tolerance = float(
            self.get_parameter("base_lateral_tolerance").value
        )
        self._base_bearing_tolerance_deg = float(
            self.get_parameter("base_bearing_tolerance_deg").value
        )
        self._base_stable_time = float(
            self.get_parameter("base_stable_time").value
        )

        if not self._source_frame:
            raise ValueError("source_frame must not be empty")
        if not self._base_frame:
            raise ValueError("base_frame must not be empty")
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
            self._tf_lookup_timeout,
        )
        if not all(isfinite(value) for value in numeric_values):
            raise ValueError("Numeric parameters must be finite")
        ApproachThresholds(
            self._target_distance,
            self._distance_tolerance,
            self._lateral_tolerance,
            self._angle_tolerance_deg,
            self._stable_time,
        ).validate()
        BaseAlignmentThresholds(
            target_forward=self._base_target_forward,
            forward_tolerance=self._base_forward_tolerance,
            lateral_tolerance=self._base_lateral_tolerance,
            bearing_tolerance_deg=self._base_bearing_tolerance_deg,
            stable_time=self._base_stable_time,
            sample_timeout=self._tag_timeout,
        ).validate()
        if self._tag_timeout < 0.0:
            raise ValueError("tag_timeout must not be negative")
        if self._publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")
        if self._filter_window < 1:
            raise ValueError("filter_window must be at least 1")
        if self._tf_lookup_timeout < 0.0:
            raise ValueError("tf_lookup_timeout must not be negative")

    def _candidate_ids(self) -> Sequence[int]:
        """Return IDs eligible for this timer cycle."""
        if self._target_tag_id >= 0:
            return (self._target_tag_id,)
        return self._allowed_tag_ids

    def _on_timer(self) -> None:
        """Look up candidates, select one, filter it, and publish one cycle."""
        now = self.get_clock().now()
        observations = [
            observation
            for tag_id in self._candidate_ids()
            if (observation := self._lookup_observation(tag_id, now)) is not None
        ]
        selection_order = list(self._candidate_ids())
        selected = select_observation(
            observations, selection_order, self._selection_mode
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
        """Return one fresh and valid TF observation, or ``None`` when unavailable."""
        tag_frame = self._tag_frame_pattern.format(id=tag_id)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._source_frame, tag_frame, Time()
            )
        except TransformException:
            # Missing transforms are expected whenever a tag leaves the camera view.
            return None

        stamp = Time.from_msg(transform.header.stamp)
        age_seconds = (now.nanoseconds - stamp.nanoseconds) / 1.0e9
        if age_seconds > self._tag_timeout:
            return None

        translation = transform.transform.translation
        if not is_valid_translation(translation.x, translation.y, translation.z):
            return None
        rotation = transform.transform.rotation
        quaternion = normalize_quaternion((rotation.x, rotation.y, rotation.z, rotation.w))
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
        """Publish only the defined loss outputs and clear temporal history."""
        self._translation_filter.reset()
        self._active_tag_id = None
        state = self._state_machine.update(None, now_seconds, None)
        self._detected_pub.publish(Bool(data=False))
        self._tag_id_pub.publish(Int32(data=-1))
        self._state_pub.publish(String(data=state.value))
        self._log_state_change(state)
        self._publish_base_lost(now_seconds)

    def _publish_base_lost(self, now_seconds: float) -> None:
        """Publish only base loss state and clear its temporal history."""
        state = self._base_state_machine.update(None, now_seconds, None)
        self._base_state_pub.publish(String(data=state.value))
        self._log_base_state_change(state)

    def _publish_base_outputs(self, camera_pose: PoseStamped) -> None:
        """Transform one fresh source pose into base_link and publish one sample."""
        stamp = camera_pose.header.stamp
        now_nanoseconds = self.get_clock().now().nanoseconds
        now_seconds = now_nanoseconds / 1.0e9
        if not is_fresh_timestamp(
            stamp.sec,
            stamp.nanosec,
            now_nanoseconds,
            self._tag_timeout,
        ):
            self.get_logger().warning(
                "Skipping base outputs for an invalid or stale camera pose stamp",
                throttle_duration_sec=5.0,
            )
            self._publish_base_lost(now_seconds)
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                camera_pose.header.frame_id,
                Time.from_msg(camera_pose.header.stamp),
                timeout=Duration(seconds=self._tf_lookup_timeout),
            )
        except TransformException as error:
            self.get_logger().warning(
                "Skipping base outputs because TF lookup failed: %s" % error,
                throttle_duration_sec=5.0,
            )
            self._publish_base_lost(now_seconds)
            return

        try:
            base_pose = transform_pose_preserving_stamp(
                camera_pose, transform, self._base_frame
            )
        except Exception as error:
            self.get_logger().warning(
                "Skipping base outputs because pose transformation failed: %s" % error,
                throttle_duration_sec=5.0,
            )
            self._publish_base_lost(now_seconds)
            return
        if base_pose is None:
            self.get_logger().warning(
                "Skipping base outputs because pose validation failed",
                throttle_duration_sec=5.0,
            )
            self._publish_base_lost(now_seconds)
            return

        try:
            metrics = compute_base_metrics(
                base_pose.pose.position.x, base_pose.pose.position.y
            )
        except ValueError:
            self._publish_base_lost(now_seconds)
            return

        state = self._base_state_machine.update(
            BaseAlignmentMeasurement(
                forward_distance=metrics.forward_distance,
                lateral_error=metrics.lateral_error,
                bearing=metrics.bearing,
                stamp_seconds=stamp.sec + stamp.nanosec / 1.0e9,
            ),
            now_seconds,
            self._active_tag_id,
        )

        self._base_pose_pub.publish(base_pose)
        self._base_forward_pub.publish(Float64(data=metrics.forward_distance))
        self._base_lateral_pub.publish(Float64(data=metrics.lateral_error))
        self._base_bearing_pub.publish(Float64(data=metrics.bearing))
        self._base_state_pub.publish(String(data=state.value))
        self._log_base_state_change(state)

    def _publish_valid(
        self,
        selected: TagObservation,
        measurement: RelativeMeasurement,
        state: ApproachState,
    ) -> None:
        """Publish pose, derived metrics, identity, detection, and state."""
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
        self._publish_base_outputs(pose)

    def _log_state_change(self, state: ApproachState) -> None:
        """Log transitions once without producing timer-rate log spam."""
        if state != self._last_logged_state:
            self.get_logger().info("Alignment state changed to %s" % state.value)
            self._last_logged_state = state

    def _log_base_state_change(self, state: ApproachState) -> None:
        """Log base state transitions without timer-rate repetition."""
        if state != self._last_logged_base_state:
            self.get_logger().info(
                "Base alignment state changed to %s" % state.value
            )
            self._last_logged_base_state = state


def main(args: Optional[List[str]] = None) -> None:
    """Run the AprilTag approach node."""
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
            # Launch or timeout can deliver another interrupt during cleanup.
            pass


if __name__ == "__main__":
    main()
