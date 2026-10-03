#!/usr/bin/env python3
"""Associate raw face observations and publish temporal Eye State results."""

from __future__ import annotations

from collections import OrderedDict
from pathlib import Path
import sys
from typing import Dict, List, Tuple

import cv2
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rescue_robot_interfaces.msg import (
    EyeStateObservation,
    EyeStateObservationArray,
    EyeStateTrackObservation,
    EyeStateTrackObservationArray,
)
from sensor_msgs.msg import Image


TOOLS_DIRECTORY = (
    Path(get_package_share_directory("rescue_robot_eye_state"))
    / "eye_state_tools"
)
if not TOOLS_DIRECTORY.is_dir():
    raise RuntimeError(f"installed Eye State helper directory missing: {TOOLS_DIRECTORY}")
sys.path.insert(0, str(TOOLS_DIRECTORY))

from eye_track import EyeStateInput, EyeTrackManager  # noqa: E402


RAW_TOPIC_DEFAULT = "/leader/survivor/eye_state/raw"
STAGE2_DEBUG_TOPIC_DEFAULT = "/leader/survivor/eye_state/debug_image"
OUTPUT_TOPIC_DEFAULT = "/leader/survivor/eye_state/stable"
DEBUG_TOPIC_DEFAULT = "/leader/survivor/eye_state/temporal_debug_image"
STATE_TO_CODE = {
    "INVALID": EyeStateTrackObservation.STATE_INVALID,
    "OPEN": EyeStateTrackObservation.STATE_OPEN,
    "CLOSED": EyeStateTrackObservation.STATE_CLOSED,
    "LOW_CONFIDENCE": EyeStateTrackObservation.STATE_LOW_CONFIDENCE,
    "NO_PREDICTION": EyeStateTrackObservation.STATE_NO_PREDICTION,
    "UNKNOWN": EyeStateTrackObservation.STATE_UNKNOWN,
}
CODE_TO_STATE = {value: key for key, value in STATE_TO_CODE.items()}


def _stamp_key(header) -> Tuple[int, int]:
    return int(header.stamp.sec), int(header.stamp.nanosec)


def _raw_state_name(code: int) -> str:
    return CODE_TO_STATE.get(int(code), "NO_PREDICTION")


def _history_ready(track, min_valid_samples: int, min_valid_coverage: float) -> bool:
    return all(
        side.valid_sample_count >= min_valid_samples
        and side.valid_coverage >= min_valid_coverage
        for side in (track.left_stable, track.right_stable)
    )


class SurvivorEyeStateTemporalNode(Node):
    """Stage 3 temporal classifier; EyeTrack IDs remain local and temporary."""

    def __init__(self) -> None:
        super().__init__("survivor_eye_state_temporal_node")
        self.declare_parameter("raw_topic", RAW_TOPIC_DEFAULT)
        self.declare_parameter("stage2_debug_image_topic", STAGE2_DEBUG_TOPIC_DEFAULT)
        self.declare_parameter("output_topic", OUTPUT_TOPIC_DEFAULT)
        self.declare_parameter("debug_image_topic", DEBUG_TOPIC_DEFAULT)
        self.declare_parameter("history_window_sec", 3.0)
        self.declare_parameter("min_valid_samples", 5)
        self.declare_parameter("min_valid_coverage", 0.5)
        self.declare_parameter("open_ratio_threshold", 0.70)
        self.declare_parameter("closed_ratio_threshold", 0.70)
        self.declare_parameter("track_timeout_sec", 1.5)
        self.declare_parameter("match_iou_threshold", 0.25)
        self.declare_parameter("match_center_distance_threshold", 0.30)

        self.raw_topic = str(self.get_parameter("raw_topic").value)
        self.stage2_debug_image_topic = str(
            self.get_parameter("stage2_debug_image_topic").value
        )
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        min_valid_samples = int(self.get_parameter("min_valid_samples").value)
        min_valid_coverage = float(
            self.get_parameter("min_valid_coverage").value
        )

        for name, value in (("raw_topic", self.raw_topic),
                            ("stage2_debug_image_topic", self.stage2_debug_image_topic),
                            ("output_topic", self.output_topic),
                            ("debug_image_topic", self.debug_image_topic)):
            if not value:
                raise ValueError(f"{name} cannot be empty")

        self.manager = EyeTrackManager(
            history_window_sec=float(self.get_parameter("history_window_sec").value),
            min_valid_samples=min_valid_samples,
            min_valid_coverage=min_valid_coverage,
            open_ratio_threshold=float(
                self.get_parameter("open_ratio_threshold").value
            ),
            closed_ratio_threshold=float(
                self.get_parameter("closed_ratio_threshold").value
            ),
            track_timeout_sec=float(self.get_parameter("track_timeout_sec").value),
            match_iou_threshold=float(
                self.get_parameter("match_iou_threshold").value
            ),
            match_center_distance_threshold=float(
                self.get_parameter("match_center_distance_threshold").value
            ),
        )
        self.min_valid_samples = min_valid_samples
        self.min_valid_coverage = min_valid_coverage
        self.bridge = CvBridge()
        self._last_stamp_ns = None
        self._raw_seen = 0

        publisher_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.output_publisher = self.create_publisher(
            EyeStateTrackObservationArray, self.output_topic, publisher_qos
        )
        self.debug_publisher = self.create_publisher(
            Image, self.debug_image_topic, qos_profile_sensor_data
        )
        self.raw_subscription = self.create_subscription(
            EyeStateObservationArray, self.raw_topic, self._raw_callback,
            publisher_qos,
        )
        self.debug_subscription = self.create_subscription(
            Image, self.stage2_debug_image_topic, self._debug_image_callback,
            qos_profile_sensor_data,
        )

        self._pending_results: OrderedDict = OrderedDict()
        self._pending_images: OrderedDict = OrderedDict()
        self.get_logger().info(
            "Eye State temporal ready: "
            f"raw={self.raw_topic}, output={self.output_topic}, "
            f"stage2_debug={self.stage2_debug_image_topic}, "
            f"debug={self.debug_image_topic}, "
            f"window={self.manager.history_window_ns / 1e9:.2f}s, "
            f"min_valid={min_valid_samples}, "
            f"thresholds=({self.manager.open_ratio_threshold:.2f},"
            f"{self.manager.closed_ratio_threshold:.2f})"
        )

    def _raw_callback(self, message: EyeStateObservationArray) -> None:
        stamp_ns = (int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec))
        if self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns:
            self.get_logger().warning(
                "raw image timestamp reset or repeated; clearing active EyeTracks"
            )
            self.manager.reset()
        self._last_stamp_ns = stamp_ns

        bboxes = []
        eye_states = []
        for observation in message.observations:
            bboxes.append((observation.x, observation.y,
                           observation.width, observation.height))
            eye_states.append((
                EyeStateInput(observation.left_valid,
                              _raw_state_name(observation.left_state),
                              float(observation.left_confidence)),
                EyeStateInput(observation.right_valid,
                              _raw_state_name(observation.right_state),
                              float(observation.right_confidence)),
            ))

        try:
            track_ids = self.manager.update(stamp_ns, bboxes, eye_states)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"invalid raw Eye State observation: {exc}")
            return

        output = EyeStateTrackObservationArray()
        output.header = message.header
        debug_observations = []
        for observation, track_id in zip(message.observations, track_ids):
            if track_id is None:
                continue
            track = self.manager.tracks[track_id]
            result = EyeStateTrackObservation()
            result.eye_track_id = track_id
            result.face_index = observation.face_index
            result.x = observation.x
            result.y = observation.y
            result.width = observation.width
            result.height = observation.height
            result.face_confidence = observation.face_confidence
            result.left_raw_valid = observation.left_valid
            result.left_raw_state = observation.left_state
            result.left_raw_confidence = observation.left_confidence
            result.left_stable_state = STATE_TO_CODE[track.left_stable.state]
            result.left_valid_samples = track.left_stable.valid_sample_count
            result.right_raw_valid = observation.right_valid
            result.right_raw_state = observation.right_state
            result.right_raw_confidence = observation.right_confidence
            result.right_stable_state = STATE_TO_CODE[track.right_stable.state]
            result.right_valid_samples = track.right_stable.valid_sample_count
            result.combined_state = STATE_TO_CODE[track.combined_state]
            result.history_ready = _history_ready(
                track, self.min_valid_samples, self.min_valid_coverage
            )
            result.history_duration_sec = min(
                track.left_stable.history_duration_sec,
                track.right_stable.history_duration_sec,
            )
            output.observations.append(result)
            debug_observations.append((observation, result))

        self.output_publisher.publish(output)
        key = _stamp_key(message.header)
        self._pending_results[key] = debug_observations
        self._pending_results.move_to_end(key)
        self._try_publish_debug(key)
        self._prune_pending()
        self._raw_seen += 1
        if self._raw_seen == 1 or self._raw_seen % 10 == 0:
            states = ", ".join(
                f"EyeTrack #{item.eye_track_id}={_raw_state_name(item.combined_state)}"
                for item in output.observations
            ) or "no faces"
            self.get_logger().info(
                f"raw frames={self._raw_seen}, tracks={len(output.observations)}; {states}"
            )

    def _debug_image_callback(self, message: Image) -> None:
        key = _stamp_key(message.header)
        self._pending_images[key] = message
        self._pending_images.move_to_end(key)
        self._try_publish_debug(key)
        self._prune_pending()

    def _try_publish_debug(self, key: Tuple[int, int]) -> None:
        if key not in self._pending_images or key not in self._pending_results:
            return
        image_message = self._pending_images.pop(key)
        observations = self._pending_results.pop(key)
        try:
            frame = self.bridge.imgmsg_to_cv2(
                image_message, desired_encoding="bgr8"
            )
        except Exception as exc:  # Keep structured output independent of debug conversion.
            self.get_logger().warning(f"cannot decode Stage 2 debug image: {exc}")
            return
        annotated = self._annotate_temporal(frame, observations)
        output = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        output.header = image_message.header
        self.debug_publisher.publish(output)

    def _prune_pending(self) -> None:
        for cache in (self._pending_images, self._pending_results):
            while len(cache) > 10:
                cache.popitem(last=False)

    @staticmethod
    def _annotate_temporal(frame, observations) -> object:
        image = frame.copy()
        if not observations:
            cv2.putText(image, "Temporal Eye State: NO FACE", (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 220, 0), 2,
                        cv2.LINE_AA)
            return image

        height, width = image.shape[:2]
        for raw, stable in observations:
            x, y, box_width, box_height = raw.x, raw.y, raw.width, raw.height
            color = (0, 210, 0) if stable.combined_state in (
                EyeStateTrackObservation.STATE_OPEN,
                EyeStateTrackObservation.STATE_CLOSED,
            ) else (0, 190, 255)
            cv2.rectangle(image, (x, y), (x + box_width, y + box_height), color, 2)
            lines = (
                f"EyeTrack #{stable.eye_track_id}  FINAL: "
                f"{_raw_state_name(stable.combined_state)} "
                f"H:{stable.history_duration_sec:.1f}s",
                f"RAW L:{_raw_state_name(raw.left_state)} {raw.left_confidence:.2f}  "
                f"R:{_raw_state_name(raw.right_state)} {raw.right_confidence:.2f}",
                f"STABLE L:{_raw_state_name(stable.left_stable_state)} "
                f"({stable.left_valid_samples})  "
                f"R:{_raw_state_name(stable.right_stable_state)} "
                f"({stable.right_valid_samples})",
            )
            text_x = min(max(4, x), max(4, width - 500))
            text_y = min(max(22, y + 20), max(22, height - 56))
            for line_index, text in enumerate(lines):
                point = (text_x, text_y + line_index * 20)
                cv2.putText(image, text, point, cv2.FONT_HERSHEY_SIMPLEX,
                            0.48, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(image, text, point, cv2.FONT_HERSHEY_SIMPLEX,
                            0.48, color, 1, cv2.LINE_AA)
        return image


def main(args=None) -> int:
    rclpy.init(args=args)
    node = SurvivorEyeStateTemporalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
