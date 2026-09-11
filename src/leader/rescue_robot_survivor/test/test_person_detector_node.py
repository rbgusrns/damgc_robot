"""Node-level tests with a fake model and no Ultralytics dependency."""

from types import SimpleNamespace
from unittest.mock import patch

from cv_bridge import CvBridge
import numpy as np
import rclpy

import rescue_robot_survivor.person_detector_node as detector_module


class FakeTensor:
    """Provide the tensor calls used by the model-result adapter."""

    def __init__(self, values):
        self._values = values

    def detach(self):
        return self

    def cpu(self):
        return self

    def tolist(self):
        return self._values


class FakeModel:
    """Return two people and one non-person in deliberately unsorted order."""

    def __init__(self):
        self.calls = []

    def predict(self, **kwargs):
        self.calls.append(kwargs)
        boxes = SimpleNamespace(
            xyxy=FakeTensor(
                [[130, 20, 180, 100], [20, 20, 70, 100], [80, 20, 120, 100]]
            ),
            conf=FakeTensor([0.9, 0.8, 0.99]),
            cls=FakeTensor([0, 0, 56]),
        )
        return [SimpleNamespace(boxes=boxes)]


class RecordingPublisher:
    """Record published ROS messages."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_model_is_loaded_once_and_source_header_is_preserved():
    model = FakeModel()
    publisher = RecordingPublisher()
    model_loads = []

    def fake_yolo(model_name):
        model_loads.append(model_name)
        return model

    rclpy.init()
    node = None
    try:
        with (
            patch.object(
                detector_module,
                "torch",
                SimpleNamespace(
                    cuda=SimpleNamespace(is_available=lambda: False)
                ),
            ),
            patch.object(detector_module, "YOLO", side_effect=fake_yolo),
            patch.object(
                detector_module.PersonDetectorNode,
                "create_publisher",
                return_value=publisher,
            ),
            patch.object(
                detector_module.PersonDetectorNode,
                "create_subscription",
                return_value=object(),
            ),
        ):
            node = detector_module.PersonDetectorNode()
            # Node parameter events also use the patched publisher factory.
            publisher.messages.clear()
            source = CvBridge().cv2_to_imgmsg(
                np.zeros((120, 200, 3), dtype=np.uint8), encoding="bgr8"
            )
            source.header.frame_id = "camera_color_optical_frame"
            source.header.stamp.sec = 123
            source.header.stamp.nanosec = 456

            node._image_callback(source)
            node._image_callback(source)

        assert model_loads == ["yolo11n.pt"]
        assert len(model.calls) == 2
        assert model.calls[0]["classes"] == [0]
        assert model.calls[0]["device"] == "cpu"
        assert len(publisher.messages) == 2
        output = publisher.messages[0]
        assert output.header.frame_id == source.header.frame_id
        assert output.header.stamp == source.header.stamp
        annotated = CvBridge().imgmsg_to_cv2(output, desired_encoding="bgr8")
        assert np.count_nonzero(annotated) > 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
