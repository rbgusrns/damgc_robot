"""Focused tests for the precomputed gripper ray filter."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from sensor_msgs.msg import CameraInfo, Image


SCRIPT = Path(__file__).parents[1] / "scripts" / "robot_self_filter.py"
SPEC = importlib.util.spec_from_file_location("robot_self_filter", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
RobotSelfFilter = MODULE.RobotSelfFilter


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_identity_tf():
    return SimpleNamespace(
        transform=SimpleNamespace(
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        )
    )


def test_fixed_box_ray_intervals_are_cached_and_filter_only_intersections():
    image = Image()
    image.width = 5
    image.height = 5
    image.step = 10
    image.encoding = "16UC1"
    image.header.frame_id = "camera"
    image.data = np.full((5, 5), 1500, dtype=np.uint16).tobytes()

    info = CameraInfo()
    info.width = 5
    info.height = 5
    info.k = [100.0, 0.0, 2.0, 0.0, 100.0, 2.0, 0.0, 0.0, 1.0]
    bounds = np.array([-0.01, 0.01, -0.01, 0.01, 1.0, 2.0])
    geometry = {}

    fake = SimpleNamespace(
        _geometry_key=None,
        _filter_geometry=None,
        _quat_rotate=RobotSelfFilter._quat_rotate,
        _rotation_matrix=RobotSelfFilter._rotation_matrix,
        tf_buffer=SimpleNamespace(lookup_transform=lambda *_args: make_identity_tf()),
        get_parameter=lambda _name: SimpleNamespace(value="base_link"),
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )
    geometry.update(
        RobotSelfFilter._prepare_filter_geometry(fake, image, info, bounds)
    )

    assert geometry["intersects"][2, 2]
    assert geometry["minimum_depth"][2, 2] == 1000.0
    assert geometry["maximum_depth"][2, 2] == 2000.0
    assert geometry["intersects"][2, 3]
    assert geometry["maximum_depth"][2, 3] <= 1000.1

    fake.camera_info = info
    fake.pub = RecordingPublisher()
    fake.get_parameter = lambda name: SimpleNamespace(
        value=True if name == "enabled" else bounds
    )
    fake._prepare_filter_geometry = lambda *_args: geometry
    fake.bridge = None
    fake.last_warn = None
    RobotSelfFilter._depth_cb(fake, image)

    filtered = np.frombuffer(fake.pub.messages[0].data, dtype=np.uint16).reshape(5, 5)
    assert filtered[2, 2] == 0
    assert filtered[2, 3] == 1500
