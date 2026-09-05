#!/usr/bin/env python3
"""
Publish Leader camera- and base-frame AprilTag observations.

The configured source frame must be a camera optical frame, where x points
right, y points down, and z points forward.  Camera-frame alignment outputs are
preserved while additional observations are transformed into ``base_link``.
"""

from math import atan2, cos, hypot, isfinite, radians, sin, sqrt
from typing import List, Optional, Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from leader_alignment_msgs.msg import LeaderAlignmentCommand
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
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
from rescue_robot_apriltag.base_alignment_logic import (
    AlignmentDecision,
    BaseAlignmentMeasurement,
    BaseAlignmentStateMachine,
    BaseAlignmentThresholds,
    ControlMode,
    compute_blind_remaining_distance,
    compute_forward_progress,
    is_blind_final_approach_eligible,
    normalize_angle,
)
from rescue_robot_apriltag.base_pose import (
    PlanarNormalMedianFilter,
    TargetGeometry,
    compute_base_metrics,
    compute_target_geometry,
    is_fresh_timestamp,
    rotate_tag_z_to_base_xy,
    select_robot_facing_normal,
    transform_pose_preserving_stamp,
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
        self._control_mode_pub = self.create_publisher(
            String, "alignment/control_mode", 10
        )
        self._normal_heading_pub = self.create_publisher(
            Float64, "alignment/tag_normal_heading", 10
        )
        self._prealign_target_pub = self.create_publisher(
            PoseStamped, "alignment/prealign_target_pose", 10
        )
        self._final_target_pub = self.create_publisher(
            PoseStamped, "alignment/final_target_pose", 10
        )
        self._control_target_pub = self.create_publisher(
            PoseStamped, "alignment/control_target_pose", 10
        )
        self._command_pub = self.create_publisher(
            LeaderAlignmentCommand, "alignment/command", 10
        )
        self._final_position_error_pub = self.create_publisher(
            Float64, "alignment/final_position_error", 10
        )
        self._final_yaw_error_pub = self.create_publisher(
            Float64, "alignment/final_yaw_error", 10
        )
        self._blind_active_pub = self.create_publisher(
            Bool, "alignment/blind_final_approach_active", 10
        )
        self._last_valid_tag_x_pub = self.create_publisher(
            Float64, "alignment/last_valid_tag_x", 10
        )
        self._blind_distance_pub = self.create_publisher(
            Float64, "alignment/blind_planned_distance", 10
        )
        self._odom_progress_pub = self.create_publisher(
            Float64, "alignment/odom_forward_progress", 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._on_odom, 10
        )

        self._translation_filter = MedianTranslationFilter(self._filter_window)
        self._normal_filter = PlanarNormalMedianFilter(self._filter_window)
        self._state_machine = ApproachStateMachine(
            ApproachThresholds(
                target_distance=self._target_distance,
                distance_tolerance=self._distance_tolerance,
                lateral_tolerance=self._lateral_tolerance,
                angle_tolerance_deg=self._angle_tolerance_deg,
                stable_time=self._stable_time,
            )
        )
        self._base_state_machine = BaseAlignmentStateMachine(
            BaseAlignmentThresholds(
                orientation_engage_distance=self._orientation_engage_distance,
                orientation_disengage_distance=(
                    self._orientation_disengage_distance
                ),
                turn_enter_error_deg=self._turn_enter_error_deg,
                turn_exit_error_deg=self._turn_exit_error_deg,
                tag_recenter_enter_deg=self._tag_recenter_enter_deg,
                tag_recenter_exit_deg=self._tag_recenter_exit_deg,
                near_normal_correction_limit_deg=(
                    self._near_normal_correction_limit_deg
                ),
                pre_align_position_tolerance=self._pre_align_position_tolerance,
                final_position_tolerance=self._final_position_tolerance,
                final_yaw_tolerance_deg=self._final_yaw_tolerance_deg,
                final_realign_yaw_error_deg=(
                    self._final_realign_yaw_error_deg
                ),
                stable_time=self._base_stable_time,
                sample_timeout=self._tag_timeout,
            )
        )
        self._active_tag_id: Optional[int] = None
        self._last_logged_state: Optional[ApproachState] = None
        self._last_logged_base_state: Optional[ApproachState] = None
        self._last_valid_tag_x: Optional[float] = None
        self._last_valid_timestamp: Optional[float] = None
        self._last_valid_yaw_error: Optional[float] = None
        self._last_valid_cross_track: Optional[float] = None
        self._last_processed_observation_stamps = {}
        self._last_odom: Optional[tuple] = None
        self._blind_active = False
        self._blind_completed = False
        self._blind_planned_distance = 0.0
        self._blind_start_odom: Optional[tuple] = None
        self._blind_previous_odom: Optional[tuple] = None
        self._blind_start_time: Optional[float] = None
        self._last_odom_progress = 0.0
        self._timer = self.create_timer(1.0 / self._publish_rate, self._on_timer)

    def _declare_parameters(self) -> None:
        """Declare startup-only Leader approach parameters."""
        self.declare_parameter("source_frame", "camera_color_optical_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tf_lookup_timeout", 0.0)
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
        self.declare_parameter("pre_align_distance", 0.30)
        self.declare_parameter("final_target_distance", 0.20)
        self.declare_parameter("orientation_engage_distance", 0.40)
        self.declare_parameter("orientation_disengage_distance", 0.43)
        self.declare_parameter("turn_enter_error_deg", 8.0)
        self.declare_parameter("turn_exit_error_deg", 3.0)
        self.declare_parameter("tag_recenter_enter_deg", 18.0)
        self.declare_parameter("tag_recenter_exit_deg", 11.0)
        self.declare_parameter("near_normal_correction_limit_deg", 6.0)
        self.declare_parameter("pre_align_position_tolerance", 0.02)
        self.declare_parameter("final_position_tolerance", 0.015)
        self.declare_parameter("final_yaw_tolerance_deg", 4.0)
        self.declare_parameter("final_realign_yaw_error_deg", 8.0)
        self.declare_parameter("base_stable_time", 0.8)
        self.declare_parameter("blind_final_approach_enabled", True)
        self.declare_parameter("blind_activation_max_tag_x", 0.30)
        self.declare_parameter("blind_max_distance", 0.12)
        self.declare_parameter("blind_last_tag_max_age", 0.25)
        self.declare_parameter("blind_handoff_max_age", 0.40)
        self.declare_parameter("blind_max_duration", 5.0)
        self.declare_parameter("odom_topic", "/leader/odom/raw")

    def _load_and_validate_parameters(self) -> None:
        """Load parameters and reject ambiguous or unsafe configurations."""
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
        self._pre_align_distance = float(
            self.get_parameter("pre_align_distance").value
        )
        self._final_target_distance = float(
            self.get_parameter("final_target_distance").value
        )
        self._orientation_engage_distance = float(
            self.get_parameter("orientation_engage_distance").value
        )
        self._orientation_disengage_distance = float(
            self.get_parameter("orientation_disengage_distance").value
        )
        self._turn_enter_error_deg = float(
            self.get_parameter("turn_enter_error_deg").value
        )
        self._turn_exit_error_deg = float(
            self.get_parameter("turn_exit_error_deg").value
        )
        self._tag_recenter_enter_deg = float(
            self.get_parameter("tag_recenter_enter_deg").value
        )
        self._tag_recenter_exit_deg = float(
            self.get_parameter("tag_recenter_exit_deg").value
        )
        self._near_normal_correction_limit_deg = float(
            self.get_parameter("near_normal_correction_limit_deg").value
        )
        self._pre_align_position_tolerance = float(
            self.get_parameter("pre_align_position_tolerance").value
        )
        self._final_position_tolerance = float(
            self.get_parameter("final_position_tolerance").value
        )
        self._final_yaw_tolerance_deg = float(
            self.get_parameter("final_yaw_tolerance_deg").value
        )
        self._final_realign_yaw_error_deg = float(
            self.get_parameter("final_realign_yaw_error_deg").value
        )
        self._base_stable_time = float(
            self.get_parameter("base_stable_time").value
        )
        self._blind_final_approach_enabled = bool(
            self.get_parameter("blind_final_approach_enabled").value
        )
        self._blind_activation_max_tag_x = float(
            self.get_parameter("blind_activation_max_tag_x").value
        )
        self._blind_max_distance = float(
            self.get_parameter("blind_max_distance").value
        )
        self._blind_last_tag_max_age = float(
            self.get_parameter("blind_last_tag_max_age").value
        )
        self._blind_handoff_max_age = float(
            self.get_parameter("blind_handoff_max_age").value
        )
        self._blind_max_duration = float(
            self.get_parameter("blind_max_duration").value
        )
        self._odom_topic = str(self.get_parameter("odom_topic").value)

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
            self._blind_activation_max_tag_x,
            self._blind_max_distance,
            self._blind_last_tag_max_age,
            self._blind_handoff_max_age,
            self._blind_max_duration,
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
        BaseAlignmentThresholds(
            orientation_engage_distance=self._orientation_engage_distance,
            orientation_disengage_distance=self._orientation_disengage_distance,
            turn_enter_error_deg=self._turn_enter_error_deg,
            turn_exit_error_deg=self._turn_exit_error_deg,
            tag_recenter_enter_deg=self._tag_recenter_enter_deg,
            tag_recenter_exit_deg=self._tag_recenter_exit_deg,
            near_normal_correction_limit_deg=(
                self._near_normal_correction_limit_deg
            ),
            pre_align_position_tolerance=self._pre_align_position_tolerance,
            final_position_tolerance=self._final_position_tolerance,
            final_yaw_tolerance_deg=self._final_yaw_tolerance_deg,
            final_realign_yaw_error_deg=self._final_realign_yaw_error_deg,
            stable_time=self._base_stable_time,
            sample_timeout=self._tag_timeout,
        ).validate()
        if not self._pre_align_distance > self._final_target_distance > 0.0:
            raise ValueError(
                "pre_align_distance must be greater than final_target_distance > 0"
            )
        if self._pre_align_distance >= self._orientation_engage_distance:
            raise ValueError(
                "orientation_engage_distance must exceed pre_align_distance"
            )
        if self._final_target_distance <= self._final_position_tolerance:
            raise ValueError(
                "final_target_distance must exceed final_position_tolerance"
            )
        if self._tag_timeout < 0.0:
            raise ValueError("tag_timeout must not be negative")
        if self._publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")
        if self._filter_window < 1:
            raise ValueError("filter_window must be at least 1")
        if self._tf_lookup_timeout < 0.0:
            raise ValueError("tf_lookup_timeout must not be negative")
        if self._blind_activation_max_tag_x <= 0.0:
            raise ValueError("blind_activation_max_tag_x must be positive")
        if self._blind_max_distance < 0.0:
            raise ValueError("blind_max_distance must not be negative")
        if self._blind_last_tag_max_age < 0.0:
            raise ValueError("blind_last_tag_max_age must not be negative")
        if self._blind_handoff_max_age <= self._blind_last_tag_max_age:
            raise ValueError(
                "blind_handoff_max_age must exceed blind_last_tag_max_age"
            )
        if self._blind_max_duration <= 0.0:
            raise ValueError("blind_max_duration must be positive")
        if not self._odom_topic:
            raise ValueError("odom_topic must not be empty")

    def _candidate_ids(self) -> Sequence[int]:
        """Return the tag IDs eligible for the current lookup cycle."""
        if self._target_tag_id >= 0:
            return (self._target_tag_id,)
        return self._allowed_tag_ids

    def _on_timer(self) -> None:
        """Look up candidates, select and filter one, then publish one cycle."""
        now = self.get_clock().now()
        now_seconds = now.nanoseconds / 1.0e9
        if getattr(self, "_blind_completed", False):
            self._publish_completed_cycle(now_seconds)
            return
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
            self._publish_lost(now_seconds)
            return

        is_new_observation = self._accept_observation_stamp(selected)
        if getattr(self, "_blind_active", False):
            if is_new_observation:
                # A fresh valid pose supersedes the blind plan immediately.
                self._clear_blind_plan()
            else:
                # A cached TF is not a reacquisition; continue the odometry plan.
                self._publish_blind_cycle(now_seconds)
                return

        if (
            not is_new_observation
            and self._final_sample_age(now_seconds) >= self._blind_last_tag_max_age
        ):
            self._publish_lost(now_seconds)
            return

        if selected.tag_id != self._active_tag_id:
            self._translation_filter.reset()
            self._normal_filter.reset()
            self._state_machine.reset()
            self._base_state_machine.reset()
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
        self._publish_valid(selected, measurement, state, is_new_observation)

    def _on_odom(self, message: Odometry) -> None:
        """Cache the existing Leader wheel odometry with local receipt time."""
        now_seconds = self.get_clock().now().nanoseconds / 1.0e9
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (
            position.x,
            position.y,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        norm = sqrt(
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        if not all(isfinite(value) for value in values) or not isfinite(norm):
            self._last_odom = None
            return
        if norm <= 1.0e-12:
            self._last_odom = None
            return
        qx, qy, qz, qw = (
            orientation.x / norm,
            orientation.y / norm,
            orientation.z / norm,
            orientation.w / norm,
        )
        yaw = atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1.0e9
        if not all(isfinite(value) for value in (now_seconds, stamp, yaw)) or stamp <= 0.0:
            self._last_odom = None
            return
        self._last_odom = (
            float(position.x),
            float(position.y),
            float(yaw),
            float(stamp),
            float(now_seconds),
        )

    def _fresh_odom(self, now_seconds: float) -> Optional[tuple]:
        """Return odometry only while both source and receipt data are fresh."""
        if self._last_odom is None:
            return None
        odom_x, odom_y, odom_yaw, stamp, received = self._last_odom
        if not all(isfinite(value) for value in self._last_odom):
            return None
        source_age = now_seconds - stamp
        receipt_age = now_seconds - received
        if (
            source_age < 0.0
            or source_age > self._blind_last_tag_max_age
            or receipt_age < 0.0
            or receipt_age > self._blind_last_tag_max_age
        ):
            return None
        return odom_x, odom_y, odom_yaw

    def _remember_last_valid_final_sample(
        self,
        metrics,
        geometry: TargetGeometry,
        decision: AlignmentDecision,
        stamp_seconds: float,
        is_new_observation: bool = True,
    ) -> None:
        """Keep only the latest sample from the visual final-approach phase."""
        if (
            is_new_observation
            and decision.mode == ControlMode.FINAL_APPROACH
            and decision.state == ApproachState.FINAL_APPROACH
        ):
            self._last_valid_tag_x = metrics.forward_distance
            self._last_valid_timestamp = stamp_seconds
            self._last_valid_yaw_error = geometry.final_yaw_error
            self._last_valid_cross_track = geometry.final_y
        elif is_new_observation:
            self._last_valid_tag_x = None
            self._last_valid_timestamp = None
            self._last_valid_yaw_error = None
            self._last_valid_cross_track = None

    def _accept_observation_stamp(self, observation: TagObservation) -> bool:
        """Accept only a strictly newer source-stamped observation as new."""
        stamp = observation.stamp_nanoseconds
        if stamp <= 0:
            return False
        previous = self._last_processed_observation_stamps.get(observation.tag_id)
        if previous is not None and stamp <= previous:
            return False
        self._last_processed_observation_stamps[observation.tag_id] = stamp
        return True

    def _final_sample_age(self, now_seconds: float) -> float:
        """Return age of the last actual visual final-approach sample."""
        if self._last_valid_timestamp is None:
            return float("inf")
        return now_seconds - self._last_valid_timestamp

    def _blind_plan(self, now_seconds: float) -> Optional[float]:
        """Build a blind plan only from a fresh visual final-approach sample."""
        if (
            self._last_valid_tag_x is None
            or self._last_valid_timestamp is None
            or self._last_valid_yaw_error is None
            or self._last_valid_cross_track is None
        ):
            return None
        odom = self._fresh_odom(now_seconds)
        if odom is None:
            return None
        phase = AlignmentDecision(
            ApproachState.FINAL_APPROACH,
            # The cached values are written only for this exact visual phase.
            ControlMode.FINAL_APPROACH,
        )
        if not is_blind_final_approach_eligible(
            enabled=self._blind_final_approach_enabled,
            phase=phase,
            last_valid_tag_x=self._last_valid_tag_x,
            last_valid_timestamp=self._last_valid_timestamp,
            now_seconds=now_seconds,
            last_valid_yaw_error=self._last_valid_yaw_error,
            last_valid_cross_track=self._last_valid_cross_track,
            final_target_distance=self._final_target_distance,
            activation_max_tag_x=self._blind_activation_max_tag_x,
            max_distance=self._blind_max_distance,
            handoff_max_age=self._blind_handoff_max_age,
            yaw_tolerance=self._final_yaw_tolerance_deg * 3.141592653589793 / 180.0,
            cross_track_tolerance=self._final_position_tolerance,
            odometry_valid=True,
        ):
            return None
        return compute_blind_remaining_distance(
            self._last_valid_tag_x,
            self._final_target_distance,
            self._blind_max_distance,
        )

    def _clear_blind_plan(self) -> None:
        """Cancel blind execution and discard its one-shot odometry snapshot."""
        self._blind_active = False
        self._blind_planned_distance = 0.0
        self._blind_start_odom = None
        self._blind_previous_odom = None
        self._blind_start_time = None
        self._last_odom_progress = 0.0

    def _publish_completed_cycle(self, now_seconds: float) -> None:
        """Hold a completed blind approach at ALIGNED with zero motion."""
        self._detected_pub.publish(Bool(data=False))
        self._tag_id_pub.publish(Int32(data=-1))
        self._state_pub.publish(String(data=ApproachState.TAG_LOST.value))
        self._publish_blind_command(
            ApproachState.ALIGNED, ControlMode.ALIGNED, now_seconds
        )

    def _publish_blind_diagnostics(self, active: bool) -> None:
        """Publish the small set of blind-fallback values used in field tests."""
        self._blind_active_pub.publish(Bool(data=active))
        self._last_valid_tag_x_pub.publish(
            Float64(data=self._last_valid_tag_x or 0.0)
        )
        self._blind_distance_pub.publish(
            Float64(data=self._blind_planned_distance)
        )
        self._odom_progress_pub.publish(Float64(data=self._last_odom_progress))

    def _publish_blind_command(
        self, state: ApproachState, mode: ControlMode, now_seconds: float
    ) -> None:
        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self._blind_planned_distance
        pose.pose.orientation.w = 1.0
        decision = AlignmentDecision(state, mode)
        self._publish_atomic_command(pose, decision)
        self._control_target_pub.publish(pose)
        self._control_mode_pub.publish(String(data=mode.value))
        self._base_state_pub.publish(String(data=state.value))
        self._publish_blind_diagnostics(
            mode == ControlMode.BLIND_FINAL_APPROACH
        )
        self._log_base_state_change(state)

    def _publish_blind_cycle(self, now_seconds: float) -> None:
        """Advance the fixed blind plan using projected odometry only."""
        if (
            self._blind_start_time is None
            or now_seconds < self._blind_start_time
            or now_seconds - self._blind_start_time > self._blind_max_duration
        ):
            self._clear_blind_plan()
            self._publish_base_lost(now_seconds)
            return
        odom = self._fresh_odom(now_seconds)
        if odom is None or self._blind_start_odom is None:
            self._clear_blind_plan()
            self._publish_base_lost(now_seconds)
            return
        progress = compute_forward_progress(
            self._blind_start_odom[0],
            self._blind_start_odom[1],
            self._blind_start_odom[2],
            odom[0],
            odom[1],
        )
        if progress is None or progress < -0.01:
            self._clear_blind_plan()
            self._publish_base_lost(now_seconds)
            return
        if self._blind_previous_odom is not None:
            step = hypot(
                odom[0] - self._blind_previous_odom[0],
                odom[1] - self._blind_previous_odom[1],
            )
            total_dx = odom[0] - self._blind_start_odom[0]
            total_dy = odom[1] - self._blind_start_odom[1]
            lateral_deviation = abs(
                -sin(self._blind_start_odom[2]) * total_dx
                + cos(self._blind_start_odom[2]) * total_dy
            )
            yaw_deviation = abs(
                normalize_angle(odom[2] - self._blind_start_odom[2])
            )
            if (
                not isfinite(step)
                or step > 0.05
                or lateral_deviation > 0.03
                or yaw_deviation > radians(12.0)
            ):
                self._clear_blind_plan()
                self._publish_base_lost(now_seconds)
                return
        self._blind_previous_odom = odom
        self._last_odom_progress = max(0.0, progress)
        if self._last_odom_progress >= self._blind_planned_distance:
            self._blind_active = False
            self._blind_completed = True
            self._publish_blind_command(
                ApproachState.ALIGNED, ControlMode.ALIGNED, now_seconds
            )
            return
        self._publish_blind_command(
            ApproachState.FINAL_APPROACH,
            ControlMode.BLIND_FINAL_APPROACH,
            now_seconds,
        )

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
        if getattr(self, "_blind_completed", False):
            self._publish_completed_cycle(now_seconds)
            return
        if getattr(self, "_blind_active", False):
            self._detected_pub.publish(Bool(data=False))
            self._tag_id_pub.publish(Int32(data=-1))
            self._state_pub.publish(String(data=ApproachState.TAG_LOST.value))
            self._publish_blind_cycle(now_seconds)
            return

        planned_distance = (
            self._blind_plan(now_seconds)
            if hasattr(self, "_blind_final_approach_enabled")
            else None
        )
        self._translation_filter.reset()
        self._normal_filter.reset()
        self._active_tag_id = None
        state = self._state_machine.update(None, now_seconds, None)
        self._detected_pub.publish(Bool(data=False))
        self._tag_id_pub.publish(Int32(data=-1))
        self._state_pub.publish(String(data=state.value))
        self._log_state_change(state)
        if planned_distance is None:
            self._publish_base_lost(now_seconds)
            return
        odom = self._fresh_odom(now_seconds)
        if odom is None:
            self._publish_base_lost(now_seconds)
            return
        self._blind_active = True
        self._blind_planned_distance = planned_distance
        self._blind_start_odom = odom
        self._blind_previous_odom = odom
        self._blind_start_time = now_seconds
        self._last_odom_progress = 0.0
        self._publish_blind_cycle(now_seconds)

    def _publish_base_lost(self, now_seconds: float) -> None:
        """Publish base-frame loss and clear its temporal alignment history."""
        self._normal_filter.reset()
        decision = self._base_state_machine.update(None, now_seconds, None)
        self._publish_atomic_command(None, decision)
        self._control_mode_pub.publish(String(data=decision.mode.value))
        self._base_state_pub.publish(String(data=decision.state.value))
        self._log_base_state_change(decision.state)

    def _publish_base_outputs(
        self, camera_pose: PoseStamped, is_new_observation: bool = True
    ) -> None:
        """Transform and publish one fresh camera pose in the configured base frame."""
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
        except Exception as error:  # Keep malformed TF data from stopping perception.
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
            orientation = base_pose.pose.orientation
            normal_x, normal_y = rotate_tag_z_to_base_xy(
                (
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                )
            )
            normal_x, normal_y = select_robot_facing_normal(
                base_pose.pose.position.x,
                base_pose.pose.position.y,
                normal_x,
                normal_y,
            )
            normal_x, normal_y = self._normal_filter.add(
                normal_x,
                normal_y,
                stamp.sec * 1_000_000_000 + stamp.nanosec,
            )
            normal_x, normal_y = select_robot_facing_normal(
                base_pose.pose.position.x,
                base_pose.pose.position.y,
                normal_x,
                normal_y,
            )
            geometry = compute_target_geometry(
                base_pose.pose.position.x,
                base_pose.pose.position.y,
                normal_x,
                normal_y,
                self._pre_align_distance,
                self._final_target_distance,
            )
        except ValueError as error:
            self.get_logger().warning(
                "Skipping base outputs because tag-normal geometry is invalid: %s"
                % error,
                throttle_duration_sec=5.0,
            )
            self._publish_base_lost(now_seconds)
            return

        decision = self._base_state_machine.update(
            BaseAlignmentMeasurement(
                tag_x=base_pose.pose.position.x,
                tag_y=base_pose.pose.position.y,
                prealign_x=geometry.prealign_x,
                prealign_y=geometry.prealign_y,
                final_x=geometry.final_x,
                final_y=geometry.final_y,
                final_yaw_error=geometry.final_yaw_error,
                stamp_seconds=(stamp.sec + stamp.nanosec / 1.0e9),
            ),
            now_seconds,
            self._active_tag_id,
        )
        remember = getattr(self, "_remember_last_valid_final_sample", None)
        if remember is not None:
            remember(
                metrics,
                geometry,
                decision,
                stamp.sec + stamp.nanosec / 1.0e9,
                is_new_observation,
            )
        if hasattr(self, "_blind_active_pub"):
            self._publish_blind_diagnostics(False)

        prealign_pose = self._make_target_pose(base_pose, geometry, prealign=True)
        final_pose = self._make_target_pose(base_pose, geometry, prealign=False)
        control_pose = self._make_control_pose(base_pose, geometry, decision)

        self._base_pose_pub.publish(base_pose)
        self._base_forward_pub.publish(
            Float64(data=metrics.forward_distance)
        )
        self._base_lateral_pub.publish(Float64(data=metrics.lateral_error))
        self._base_bearing_pub.publish(Float64(data=metrics.bearing))
        self._normal_heading_pub.publish(
            Float64(data=atan2(geometry.normal_y, geometry.normal_x))
        )
        self._prealign_target_pub.publish(prealign_pose)
        self._final_target_pub.publish(final_pose)
        self._final_position_error_pub.publish(
            Float64(data=geometry.final_position_error)
        )
        self._final_yaw_error_pub.publish(
            Float64(data=geometry.final_yaw_error)
        )
        # Publish the authoritative sample, mode, then state as one generation.
        self._publish_atomic_command(control_pose, decision)
        self._control_target_pub.publish(control_pose)
        self._control_mode_pub.publish(String(data=decision.mode.value))
        self._base_state_pub.publish(String(data=decision.state.value))
        self._log_base_state_change(decision.state)

    def _publish_atomic_command(
        self,
        control_pose: Optional[PoseStamped],
        decision: AlignmentDecision,
    ) -> None:
        """Publish pose, mode, and state from one evaluated decision atomically."""
        command = LeaderAlignmentCommand()
        if control_pose is not None:
            command.header = control_pose.header
            command.target_pose = control_pose.pose
        else:
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = self._base_frame
        command.control_mode = decision.mode.value
        command.alignment_state = decision.state.value
        self._command_pub.publish(command)

    def _make_control_pose(
        self,
        base_pose: PoseStamped,
        geometry: TargetGeometry,
        decision: AlignmentDecision,
    ) -> PoseStamped:
        """Encode the state machine's active planar command-space target."""
        target = PoseStamped()
        target.header = base_pose.header
        target.pose.position.x = decision.control_x
        target.pose.position.y = decision.control_y
        target.pose.position.z = 0.0
        target.pose.orientation.z = sin(geometry.target_yaw / 2.0)
        target.pose.orientation.w = cos(geometry.target_yaw / 2.0)
        return target

    def _make_target_pose(
        self,
        base_pose: PoseStamped,
        geometry: TargetGeometry,
        *,
        prealign: bool,
    ) -> PoseStamped:
        """Encode one desired planar robot pose relative to current base_link."""
        target = PoseStamped()
        target.header = base_pose.header
        target.pose.position.x = (
            geometry.prealign_x if prealign else geometry.final_x
        )
        target.pose.position.y = (
            geometry.prealign_y if prealign else geometry.final_y
        )
        target.pose.position.z = 0.0
        target.pose.orientation.z = sin(geometry.target_yaw / 2.0)
        target.pose.orientation.w = cos(geometry.target_yaw / 2.0)
        return target

    def _publish_valid(
        self,
        selected: TagObservation,
        measurement: RelativeMeasurement,
        state: ApproachState,
        is_new_observation: bool = True,
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
        self._publish_base_outputs(pose, is_new_observation)

    def _log_state_change(self, state: ApproachState) -> None:
        """Log state transitions once instead of logging at timer frequency."""
        if state != self._last_logged_state:
            self.get_logger().info("Alignment state changed to %s" % state.value)
            self._last_logged_state = state

    def _log_base_state_change(self, state: ApproachState) -> None:
        """Log base-frame state transitions once instead of every timer cycle."""
        if state != self._last_logged_base_state:
            self.get_logger().info(
                "Base alignment state changed to %s" % state.value
            )
            self._last_logged_base_state = state


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
