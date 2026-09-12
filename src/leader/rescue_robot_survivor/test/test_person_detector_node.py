"""Node-level tests with a fake model and no Ultralytics dependency."""

from types import SimpleNamespace
from unittest.mock import patch

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image

import rescue_robot_survivor.person_detector_node as detector_module
from rescue_robot_survivor.geometry_logic import CameraPoint


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
    debug_publisher = RecordingPublisher()
    positions_publisher = RecordingPublisher()
    parameter_publisher = RecordingPublisher()
    model_loads = []

    def fake_yolo(model_name):
        model_loads.append(model_name)
        return model

    def publisher_for(message_type, *_args, **_kwargs):
        if message_type is Image:
            return debug_publisher
        if message_type is PoseArray:
            return positions_publisher
        return parameter_publisher

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
                side_effect=publisher_for,
            ),
            patch.object(
                detector_module.PersonDetectorNode,
                "create_subscription",
                return_value=object(),
            ),
        ):
            node = detector_module.PersonDetectorNode()
            debug_publisher.messages.clear()
            positions_publisher.messages.clear()
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
        assert len(debug_publisher.messages) == 2
        assert len(positions_publisher.messages) == 2
        output = debug_publisher.messages[0]
        assert output.header.frame_id == source.header.frame_id
        assert output.header.stamp == source.header.stamp
        annotated = CvBridge().imgmsg_to_cv2(output, desired_encoding="bgr8")
        assert np.count_nonzero(annotated) > 0
        assert positions_publisher.messages[0].poses == []
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_synthetic_depth_and_camera_info_publish_ordered_camera_positions():
    model = FakeModel()
    debug_publisher = RecordingPublisher()
    positions_publisher = RecordingPublisher()
    parameter_publisher = RecordingPublisher()

    def publisher_for(message_type, *_args, **_kwargs):
        if message_type is Image:
            return debug_publisher
        if message_type is PoseArray:
            return positions_publisher
        return parameter_publisher

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
            patch.object(detector_module, "YOLO", return_value=model),
            patch.object(
                detector_module.PersonDetectorNode,
                "create_publisher",
                side_effect=publisher_for,
            ),
            patch.object(
                detector_module.PersonDetectorNode,
                "create_subscription",
                return_value=object(),
            ),
        ):
            node = detector_module.PersonDetectorNode()
            debug_publisher.messages.clear()
            positions_publisher.messages.clear()

            camera_info = CameraInfo()
            camera_info.header.frame_id = "camera_color_optical_frame"
            camera_info.width = 200
            camera_info.height = 120
            camera_info.p = [
                100.0,
                0.0,
                100.0,
                0.0,
                0.0,
                100.0,
                60.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ]
            node._camera_info_callback(camera_info)

            bridge = CvBridge()
            depth = bridge.cv2_to_imgmsg(
                np.full((120, 200), 2000, dtype=np.uint16),
                encoding="16UC1",
            )
            depth.header.frame_id = "camera_color_optical_frame"
            depth.header.stamp.sec = 123
            depth.header.stamp.nanosec = 456
            node._depth_callback(depth)

            source = bridge.cv2_to_imgmsg(
                np.zeros((120, 200, 3), dtype=np.uint8), encoding="bgr8"
            )
            source.header.frame_id = "camera_color_optical_frame"
            source.header.stamp.sec = 123
            source.header.stamp.nanosec = 456
            node._image_callback(source)

        assert len(debug_publisher.messages) == 1
        assert len(positions_publisher.messages) == 1
        positions = positions_publisher.messages[0]
        assert positions.header.stamp == source.header.stamp
        assert positions.header.frame_id == source.header.frame_id
        assert len(positions.poses) == 2
        assert positions.poses[0].position.x < 0.0
        assert positions.poses[1].position.x > 0.0
        assert positions.poses[0].position.z == 2.0
        assert positions.poses[1].position.z == 2.0
        assert all(pose.orientation.w == 1.0 for pose in positions.poses)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_pose_array_omits_invalid_person_without_origin_placeholder():
    positions_publisher = RecordingPublisher()
    node = SimpleNamespace(_positions_publisher=positions_publisher)
    source = Image()
    source.header.frame_id = "camera_color_optical_frame"
    source.header.stamp.sec = 42

    detector_module.PersonDetectorNode._publish_positions(
        node,
        {
            1: CameraPoint(-0.5, 0.1, 2.0),
            2: None,
            3: CameraPoint(0.6, 0.2, 3.0),
        },
        source,
    )

    positions = positions_publisher.messages[0]
    assert positions.header == source.header
    assert len(positions.poses) == 2
    assert [pose.position.x for pose in positions.poses] == [-0.5, 0.6]
    assert all(pose.orientation.w == 1.0 for pose in positions.poses)
