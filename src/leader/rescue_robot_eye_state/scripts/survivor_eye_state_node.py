#!/usr/bin/env python3
"""Standalone ROS 2 face/eye ROI and OPEN/CLOSED inference node."""

from __future__ import annotations

from dataclasses import dataclass
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image


TOOLS_DIRECTORY = (
    Path(get_package_share_directory("rescue_robot_eye_state"))
    / "eye_state_tools"
)
if not TOOLS_DIRECTORY.is_dir():
    raise RuntimeError(f"installed Eye State helper directory missing: {TOOLS_DIRECTORY}")
sys.path.insert(0, str(TOOLS_DIRECTORY))

from eye_classifier import EyeStateClassifier, classify_valid_eye_rois  # noqa: E402
from face_eye_roi import ExtractorConfig, FaceEyeROIExtractor  # noqa: E402


DEFAULT_MODEL_PATH = Path(
    "/home/maze/eye_state_runs/eye_state/baseline/weights/eye_state_best.pt"
)
DEBUG_IMAGE_TOPIC = "/leader/survivor/eye_state/debug_image"
LATEST_IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
BBox = Tuple[int, int, int, int]


@dataclass(frozen=True)
class EyeStateSideResult:
    """Structured, frame-local result for one eye."""

    valid: bool
    state: str
    confidence: Optional[float]
    bbox: Optional[BBox]
    reason: str = ""


@dataclass(frozen=True)
class FaceEyeStateResult:
    """Frame-local face result; index is not a persistent Survivor ID."""

    index: int
    bbox: BBox
    face_confidence: float
    left: EyeStateSideResult
    right: EyeStateSideResult


@dataclass(frozen=True)
class EyeStateFrameResult:
    """Raw structured results retained in memory for later ROS interfaces."""

    stamp_sec: int
    stamp_nanosec: int
    faces: Tuple[FaceEyeStateResult, ...]


def _side_result(eye, prediction) -> EyeStateSideResult:
    if not eye.valid:
        return EyeStateSideResult(
            valid=False,
            state="INVALID",
            confidence=None,
            bbox=eye.bbox,
            reason=eye.reason,
        )
    if prediction is None:
        return EyeStateSideResult(
            valid=True,
            state="NO_PREDICTION",
            confidence=None,
            bbox=eye.bbox,
            reason="classifier_returned_no_prediction",
        )
    return EyeStateSideResult(
        valid=True,
        state=prediction.state,
        confidence=prediction.confidence,
        bbox=eye.bbox,
    )


def make_frame_result(stamp, faces, predictions) -> EyeStateFrameResult:
    """Build a reusable result object with the source image timestamp."""
    face_results = []
    for index, face in enumerate(faces):
        face_results.append(FaceEyeStateResult(
            index=index,
            bbox=face.bbox,
            face_confidence=face.confidence,
            left=_side_result(
                face.left_eye, predictions.get((index, "left"))
            ),
            right=_side_result(
                face.right_eye, predictions.get((index, "right"))
            ),
        ))
    return EyeStateFrameResult(
        stamp_sec=int(stamp.sec),
        stamp_nanosec=int(stamp.nanosec),
        faces=tuple(face_results),
    )


def annotate_frame(frame, faces, predictions, show_landmarks: bool):
    """Draw frame-local faces, eye ROIs, results, and optional landmarks."""
    output = frame.copy()
    for index, face in enumerate(faces):
        x, y, width, height = face.bbox
        cv2.rectangle(output, (x, y), (x + width, y + height), (0, 220, 0), 2)
        cv2.putText(
            output,
            f"Face {index} ({face.confidence:.2f})",
            (x, max(18, y - 7)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 220, 0),
            1,
            cv2.LINE_AA,
        )
        side_labels = []
        for eye, prefix, color in (
            (face.left_eye, "L", (255, 0, 255)),
            (face.right_eye, "R", (0, 255, 255)),
        ):
            prediction = predictions.get((index, eye.side)) if eye.valid else None
            if not eye.valid:
                text = f"{prefix}: INVALID ({eye.reason})"
            elif prediction is None:
                text = f"{prefix}: ERROR"
            else:
                text = f"{prefix}: {prediction.state} {prediction.confidence:.2f}"
            if eye.bbox is not None:
                ex, ey, ew, eh = eye.bbox
                cv2.rectangle(output, (ex, ey), (ex + ew, ey + eh), color, 2)
            side_labels.append((text, color))

        # Keep left/right result text on separate rows. Eye centers can be so
        # close that anchoring both labels beside their boxes makes them overlap.
        frame_height, frame_width = output.shape[:2]
        if y + height + 42 < frame_height:
            label_baselines = (y + height + 18, y + height + 38)
        else:
            label_baselines = (max(15, y - 28), max(15, y - 8))
        label_x = max(2, min(x, frame_width - 260))
        for (text, color), text_y in zip(side_labels, label_baselines):
            cv2.putText(
                output,
                text,
                (label_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                color,
                1,
                cv2.LINE_AA,
            )
        if show_landmarks:
            for point in face.landmarks:
                if point is not None:
                    px, py = (int(round(value)) for value in point)
                    cv2.circle(output, (px, py), 2, (255, 255, 0), -1)
    if not faces:
        cv2.putText(
            output, "NO_FACE", (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
            0.8, (0, 0, 255), 2, cv2.LINE_AA,
        )
    return output


class SurvivorEyeStateNode(Node):
    """Keep only the newest image and run Eye State at a bounded rate."""

    def __init__(self):
        super().__init__("survivor_eye_state_node")
        self.declare_parameter("image_topic", "/leader/camera/color/image_raw")
        self.declare_parameter("model_path", str(DEFAULT_MODEL_PATH))
        self.declare_parameter(
            "yunet_model_path",
            str(Path.home() / ".cache/damgc-eye-state/face_detection_yunet_2022mar.onnx"),
        )
        self.declare_parameter("face_conf_threshold", 0.60)
        self.declare_parameter("eye_conf_threshold", 0.60)
        self.declare_parameter("min_eye_width", 24)
        self.declare_parameter("min_eye_height", 24)
        self.declare_parameter("inference_rate_hz", 5.0)
        self.declare_parameter("preprocess_mode", "gray")
        self.declare_parameter("show_landmarks", True)
        self.declare_parameter("device", "cpu")

        image_topic = str(self.get_parameter("image_topic").value)
        model_path = str(self.get_parameter("model_path").value)
        yunet_model_path = str(self.get_parameter("yunet_model_path").value)
        face_threshold = float(self.get_parameter("face_conf_threshold").value)
        eye_threshold = float(self.get_parameter("eye_conf_threshold").value)
        min_eye_width = int(self.get_parameter("min_eye_width").value)
        min_eye_height = int(self.get_parameter("min_eye_height").value)
        self.inference_rate_hz = float(
            self.get_parameter("inference_rate_hz").value
        )
        preprocess_mode = str(self.get_parameter("preprocess_mode").value)
        self.show_landmarks = bool(self.get_parameter("show_landmarks").value)
        device = str(self.get_parameter("device").value)

        if not image_topic:
            raise ValueError("image_topic must not be empty")
        if not 0.0 < self.inference_rate_hz <= 30.0:
            raise ValueError("inference_rate_hz must be in (0, 30]")
        if min_eye_width < 1 or min_eye_height < 1:
            raise ValueError("minimum eye ROI dimensions must be positive")

        self.bridge = CvBridge()
        config = ExtractorConfig(
            face_confidence_threshold=face_threshold,
            min_eye_width=min_eye_width,
            min_eye_height=min_eye_height,
        )
        # Both models are initialized once for the lifetime of this node.
        self.extractor = FaceEyeROIExtractor(
            yunet_model_path,
            config,
        )
        self.classifier = EyeStateClassifier(
            model_path=model_path,
            device=device,
            preprocessing_mode=preprocess_mode,
            min_confidence=eye_threshold,
        )

        self.debug_publisher = self.create_publisher(
            Image, DEBUG_IMAGE_TOPIC, qos_profile_sensor_data
        )
        self.image_subscription = self.create_subscription(
            Image, image_topic, self._image_callback, LATEST_IMAGE_QOS
        )
        self._condition = threading.Condition()
        self._latest_message = None
        self._stopping = False
        self._latest_result: Optional[EyeStateFrameResult] = None
        self._last_error_log_time = 0.0
        self._worker = threading.Thread(
            target=self._process_latest_loop,
            name="eye-state-inference",
            daemon=True,
        )
        self._worker.start()
        self.get_logger().info(
            "Eye State ready: "
            f"image_topic={image_topic}, debug_topic={DEBUG_IMAGE_TOPIC}, "
            f"model={model_path}, classes={self.classifier.names}, "
            f"device={self.classifier.device}, "
            f"rate_limit={self.inference_rate_hz:.2f} Hz, "
            f"preprocess={preprocess_mode}"
        )

    @property
    def latest_result(self) -> Optional[EyeStateFrameResult]:
        """Most recent timestamped raw result for future consumers."""
        with self._condition:
            return self._latest_result

    def _image_callback(self, message: Image) -> None:
        # Replacing one slot prevents a slow inference worker from building a
        # queue of stale camera frames.
        with self._condition:
            self._latest_message = message
            self._condition.notify()

    def _process_latest_loop(self) -> None:
        interval = 1.0 / self.inference_rate_hz
        next_allowed = 0.0
        while True:
            with self._condition:
                while not self._stopping and self._latest_message is None:
                    self._condition.wait()
                if self._stopping:
                    return
                wait_seconds = next_allowed - time.monotonic()
                if wait_seconds > 0.0:
                    self._condition.wait(timeout=wait_seconds)
                    continue
                message = self._latest_message
                self._latest_message = None
            try:
                self._process_message(message)
            except Exception as exc:  # keep the image subscription alive
                now = time.monotonic()
                if now - self._last_error_log_time >= 5.0:
                    self.get_logger().error(
                        f"Eye State frame failed: {type(exc).__name__}: {exc}"
                    )
                    self._last_error_log_time = now
            next_allowed = time.monotonic() + interval

    def _process_message(self, message: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        faces = self.extractor.extract(frame)
        predictions = classify_valid_eye_rois(faces, self.classifier)
        result = make_frame_result(message.header.stamp, faces, predictions)
        with self._condition:
            self._latest_result = result

        annotated = annotate_frame(
            frame, faces, predictions, self.show_landmarks
        )
        debug_message = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        debug_message.header = message.header
        self.debug_publisher.publish(debug_message)

    def destroy_node(self):
        with self._condition:
            self._stopping = True
            self._condition.notify_all()
        if self._worker.is_alive():
            self._worker.join()
        return super().destroy_node()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = SurvivorEyeStateNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(
                f"Could not start survivor_eye_state_node: "
                f"{type(exc).__name__}: {exc}"
            )
        else:
            print(
                f"Could not start survivor_eye_state_node: "
                f"{type(exc).__name__}: {exc}",
                file=sys.stderr,
            )
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
