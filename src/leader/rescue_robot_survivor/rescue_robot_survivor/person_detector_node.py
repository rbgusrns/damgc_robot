"""ROS 2 node that publishes YOLO11n person detections as a debug image."""

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
        self._publisher = self.create_publisher(
            Image, self._debug_image_topic, IMAGE_QOS
        )
        self._subscription = self.create_subscription(
            Image, self._image_topic, self._image_callback, IMAGE_QOS
        )

        self.get_logger().info(f"Model: {self._model_name}")
        self.get_logger().info(f"Input image topic: {self._image_topic}")
        self.get_logger().info(
            f"Debug image topic: {self._debug_image_topic}"
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
            draw_person_detections(image, people)
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
