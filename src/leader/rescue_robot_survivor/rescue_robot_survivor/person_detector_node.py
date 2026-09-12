"""ROS 2 node that publishes YOLO11n detections with aligned depth."""

import threading
from collections import deque
from typing import Any, List

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image

from rescue_robot_survivor.detection_logic import (
    Detection,
    draw_person_detections,
    prepare_person_detections,
)
from rescue_robot_survivor.depth_logic import estimate_person_distance

try:
    import torch
except ImportError:  # Optional for package build and pure tests.
    torch = None

try:
    from ultralytics import YOLO
except ImportError:  # Reported as an actionable startup error.
    YOLO = None


IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class PersonDetectorNode(Node):
    """Detect every COCO person and publish a frame-local annotated image."""

    def __init__(self) -> None:
        """Initialize parameters, the model, and ROS endpoints once."""
        super().__init__("person_detector")
        self._image_topic = str(
            self.declare_parameter(
                "image_topic", "/leader/camera/color/image_rect"
            ).value
        )
        self._debug_image_topic = str(
            self.declare_parameter(
                "debug_image_topic", "/leader/survivor/debug_image"
            ).value
        )
        self._model_name = str(
            self.declare_parameter("model_name", "yolo11n.pt").value
        )
        self._confidence_threshold = float(
            self.declare_parameter("confidence_threshold", 0.5).value
        )
        self._requested_device = str(
            self.declare_parameter("device", "auto").value
        ).strip()
        self._aligned_depth_topic = str(
            self.declare_parameter(
                "aligned_depth_topic",
                "/leader/camera/aligned_depth_to_color/image_raw",
            ).value
        )
        self._depth_roi_width_ratio = float(
            self.declare_parameter("depth_roi_width_ratio", 0.25).value
        )
        self._depth_roi_height_ratio = float(
            self.declare_parameter("depth_roi_height_ratio", 0.25).value
        )
        self._min_depth_m = float(
            self.declare_parameter("min_depth_m", 0.2).value
        )
        self._max_depth_m = float(
            self.declare_parameter("max_depth_m", 6.0).value
        )
        self._min_valid_depth_pixels = int(
            self.declare_parameter("min_valid_depth_pixels", 20).value
        )
        self._depth_scale_m_per_unit = float(
            self.declare_parameter("depth_scale_m_per_unit", 0.001).value
        )
        self._show_depth_roi = bool(
            self.declare_parameter("show_depth_roi", False).value
        )
        self._sync_slop_sec = float(
            self.declare_parameter("sync_slop_sec", 0.05).value
        )
        self._sync_queue_size = int(
            self.declare_parameter("sync_queue_size", 5).value
        )

        self._validate_parameters()
        if torch is None:
            raise RuntimeError(
                "PyTorch is not installed. Install the NVIDIA "
                "JetPack-compatible "
                "PyTorch build; do not replace it with a generic PyPI wheel."
            )
        if YOLO is None:
            raise RuntimeError(
                "Ultralytics is not installed. Prepare the Jetson-compatible "
                "PyTorch environment first, then install Ultralytics without "
                "allowing it to replace torch or torchvision."
            )

        self._device = self._select_device(self._requested_device)
        self._auto_cpu_fallback_used = False
        self._bridge = CvBridge()
        # Model construction happens exactly once, never in the image callback.
        self._model = YOLO(self._model_name)
        self._depth_messages = deque(maxlen=self._sync_queue_size)
        self._depth_lock = threading.Lock()
        self._publisher = self.create_publisher(
            Image, self._debug_image_topic, IMAGE_QOS
        )
        self._image_subscription = self.create_subscription(
            Image, self._image_topic, self._image_callback, IMAGE_QOS
        )
        self._depth_subscription = self.create_subscription(
            Image, self._aligned_depth_topic, self._depth_callback, IMAGE_QOS
        )

        self.get_logger().info(f"Model: {self._model_name}")
        self.get_logger().info(f"Input image topic: {self._image_topic}")
        self.get_logger().info(
            f"Debug image topic: {self._debug_image_topic}"
        )
        self.get_logger().info(
            f"Aligned depth topic: {self._aligned_depth_topic}"
        )
        self.get_logger().info(
            f"Confidence threshold: {self._confidence_threshold:.3f}"
        )
        self.get_logger().info(f"Selected device: {self._device}")

    def _validate_parameters(self) -> None:
        if not self._image_topic:
            raise ValueError("image_topic must not be empty")
        if not self._debug_image_topic:
            raise ValueError("debug_image_topic must not be empty")
        if not self._model_name:
            raise ValueError("model_name must not be empty")
        if not 0.0 <= self._confidence_threshold <= 1.0:
            raise ValueError(
                "confidence_threshold must be between 0.0 and 1.0"
            )
        if not self._requested_device:
            raise ValueError("device must not be empty")
        if not self._aligned_depth_topic:
            raise ValueError("aligned_depth_topic must not be empty")
        if not 0.0 < self._depth_roi_width_ratio <= 1.0:
            raise ValueError("depth_roi_width_ratio must be in (0.0, 1.0]")
        if not 0.0 < self._depth_roi_height_ratio <= 1.0:
            raise ValueError("depth_roi_height_ratio must be in (0.0, 1.0]")
        if self._min_depth_m < 0.0 or self._max_depth_m <= self._min_depth_m:
            raise ValueError("depth range is invalid")
        if self._min_valid_depth_pixels <= 0:
            raise ValueError("min_valid_depth_pixels must be positive")
        if self._depth_scale_m_per_unit <= 0.0:
            raise ValueError("depth_scale_m_per_unit must be positive")
        if self._sync_slop_sec <= 0.0:
            raise ValueError("sync_slop_sec must be positive")
        if self._sync_queue_size <= 0:
            raise ValueError("sync_queue_size must be positive")

    @staticmethod
    def _select_device(requested_device: str) -> str:
        if requested_device.lower() != "auto":
            return requested_device
        return "0" if torch.cuda.is_available() else "cpu"

    def _predict(self, image: Any) -> Any:
        try:
            return self._model.predict(
                source=image,
                conf=self._confidence_threshold,
                classes=[0],
                device=self._device,
                verbose=False,
            )
        except Exception:
            if (
                self._requested_device.lower() == "auto"
                and self._device != "cpu"
                and not self._auto_cpu_fallback_used
            ):
                self._auto_cpu_fallback_used = True
                self._device = "cpu"
                self.get_logger().warning(
                    "CUDA inference failed in auto mode; retrying on CPU."
                )
                return self._model.predict(
                    source=image,
                    conf=self._confidence_threshold,
                    classes=[0],
                    device=self._device,
                    verbose=False,
                )
            raise

    @staticmethod
    def _extract_detections(results: Any) -> List[Detection]:
        if not results or results[0].boxes is None:
            return []
        boxes = results[0].boxes
        coordinates = boxes.xyxy.detach().cpu().tolist()
        confidences = boxes.conf.detach().cpu().tolist()
        classes = boxes.cls.detach().cpu().tolist()
        return [
            Detection(*coordinate, float(confidence), int(class_id))
            for coordinate, confidence, class_id in zip(
                coordinates, confidences, classes
            )
        ]

    def _publish_image(self, image: Any, source_message: Image) -> None:
        debug_message = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        debug_message.header = source_message.header
        self._publisher.publish(debug_message)

    @staticmethod
    def _stamp_to_nanoseconds(message: Image) -> int:
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def _depth_callback(self, message: Image) -> None:
        """Cache recent aligned depth for non-blocking RGB processing."""
        with self._depth_lock:
            self._depth_messages.append(message)

    def _get_matching_depth(self, rgb_message: Image):
        with self._depth_lock:
            depth_messages = tuple(self._depth_messages)
        if not depth_messages:
            return None
        rgb_stamp = self._stamp_to_nanoseconds(rgb_message)
        if rgb_stamp == 0:
            return None
        matching = [
            (abs(rgb_stamp - self._stamp_to_nanoseconds(depth)), depth)
            for depth in depth_messages
            if self._stamp_to_nanoseconds(depth) != 0
        ]
        if not matching:
            return None
        delta_ns, depth_message = min(matching, key=lambda item: item[0])
        delta_sec = delta_ns / 1e9
        if delta_sec > self._sync_slop_sec:
            self.get_logger().warning(
                "RGB/aligned depth timestamp difference is "
                f"{delta_sec:.3f}s; using N/A.",
                throttle_duration_sec=5.0,
            )
            return None
        return depth_message

    def _estimate_distances(self, people, rgb_message: Image, rgb_shape):
        distances = {number: None for number in range(1, len(people) + 1)}
        rois = {}
        depth_message = self._get_matching_depth(rgb_message)
        if depth_message is None:
            return distances, rois
        try:
            depth_image = self._bridge.imgmsg_to_cv2(
                depth_message, desired_encoding="passthrough"
            )
        except CvBridgeError as error:
            self.get_logger().warning(
                f"Aligned depth conversion failed: {error}",
                throttle_duration_sec=5.0,
            )
            return distances, rois
        if depth_image.ndim != 2:
            self.get_logger().warning(
                f"Aligned depth must be single-channel, got {depth_image.shape}.",
                throttle_duration_sec=5.0,
            )
            return distances, rois
        if depth_image.shape[:2] != rgb_shape[:2]:
            self.get_logger().warning(
                f"RGB/depth resolution mismatch: RGB {rgb_shape[:2]}, "
                f"depth {depth_image.shape[:2]}; using N/A.",
                throttle_duration_sec=5.0,
            )
            return distances, rois
        for number, person in enumerate(people, start=1):
            try:
                estimate, roi = estimate_person_distance(
                    depth_image,
                    depth_message.encoding,
                    (person.x1, person.y1, person.x2, person.y2),
                    self._depth_roi_width_ratio,
                    self._depth_roi_height_ratio,
                    self._depth_scale_m_per_unit,
                    self._min_depth_m,
                    self._max_depth_m,
                    self._min_valid_depth_pixels,
                )
            except (TypeError, ValueError) as error:
                self.get_logger().warning(
                    f"Depth estimate failed: {error}",
                    throttle_duration_sec=5.0,
                )
                continue
            distances[number] = estimate.distance_m
            if roi is not None:
                rois[number] = roi
        return distances, rois

    def _image_callback(self, message: Image) -> None:
        try:
            image = self._bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8"
            )
        except CvBridgeError as error:
            self.get_logger().error(
                f"RGB image conversion failed: {error}",
                throttle_duration_sec=5.0,
            )
            return

        try:
            results = self._predict(image)
            detections = self._extract_detections(results)
            people = prepare_person_detections(
                detections,
                self._confidence_threshold,
                image.shape[1],
                image.shape[0],
            )
            # Depth is optional: RGB detection and debug output remain alive
            # while the aligned stream is absent or temporarily mismatched.
            distances, rois = self._estimate_distances(people, message, image.shape)
            draw_person_detections(
                image,
                people,
                distances=distances,
                rois=rois,
                show_depth_roi=self._show_depth_roi,
            )
        except Exception as error:
            self.get_logger().error(
                f"YOLO inference failed: {error}",
                throttle_duration_sec=5.0,
            )
        self._publish_image(image, message)


def main(args=None) -> None:
    """Run the person detector until ROS shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = PersonDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        rclpy.logging.get_logger("person_detector").fatal(str(error))
        raise SystemExit(1) from error
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
