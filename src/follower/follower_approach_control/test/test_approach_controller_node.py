"""ROS-message boundary tests for the Follower approach controller."""

from math import atan2, nan
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool

import follower_approach_control.approach_controller_node as node_module
from follower_approach_control.approach_controller_logic import PlanarCommand
from follower_approach_control.approach_controller_node import ApproachControllerNode


def make_pose() -> PoseStamped:
    """Create a finite forward base pose."""
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.header.stamp.sec = 10
    pose.pose.position.x = 0.35
    pose.pose.position.y = 0.04
    pose.pose.orientation.w = 1.0
    return pose


class RecordingPublisher:
    """Publisher double that records messages."""

    def __init__(self) -> None:
        self.messages = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


def test_twist_populates_only_differential_drive_axes() -> None:
    twist = ApproachControllerNode._to_twist(PlanarCommand(0.04, -0.10))
    assert twist.linear.x == 0.04
    assert twist.angular.z == -0.10
    assert twist.linear.y == 0.0
    assert twist.linear.z == 0.0
    assert twist.angular.x == 0.0
    assert twist.angular.y == 0.0


def test_pose_validation_accepts_finite_forward_pose() -> None:
    assert ApproachControllerNode._pose_is_valid(make_pose())


@pytest.mark.parametrize("invalid", ["nan", "nonforward", "quaternion"])
def test_pose_validation_rejects_invalid_pose(invalid: str) -> None:
    pose = make_pose()
    if invalid == "nan":
        pose.pose.position.y = nan
    elif invalid == "nonforward":
        pose.pose.position.x = 0.0
    else:
        pose.pose.orientation.w = 0.0
    assert not ApproachControllerNode._pose_is_valid(pose)


def test_pose_callback_recomputes_all_metrics_from_one_pose(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(node_module.time, "monotonic", lambda: 20.0)
    harness = SimpleNamespace(
        _detected=True,
        _selected_tag_id=0,
        _target_tag_id=0,
        _base_frame="base_link",
        _pose_timeout=0.35,
        _latest_pose_generation=4,
        _coherent_generation=4,
        _coherent_state="APPROACH",
        _invalidate_sample=lambda: None,
        _pose_is_valid=ApproachControllerNode._pose_is_valid,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=10_200_000_000)
        ),
    )

    ApproachControllerNode._on_pose(harness, make_pose())

    assert harness._measurement.forward_distance == pytest.approx(0.35)
    assert harness._measurement.lateral_error == pytest.approx(0.04)
    assert harness._measurement.bearing == pytest.approx(atan2(0.04, 0.35))
    assert harness._latest_pose_generation == 5
    assert harness._coherent_generation is None
    assert harness._coherent_state is None


def test_enable_transition_discards_cache_and_publishes_zero() -> None:
    publisher = RecordingPublisher()
    invalidations = []
    harness = SimpleNamespace(
        _enabled=False,
        _invalidate_sample=lambda: invalidations.append(True),
        _raw_pub=publisher,
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )
    request = SetBool.Request(data=True)
    response = SetBool.Response()

    result = ApproachControllerNode._on_enable(harness, request, response)

    assert harness._enabled is True
    assert invalidations == [True]
    assert result.success is True
    assert "fresh coherent sample" in result.message
    assert len(publisher.messages) == 1
    assert publisher.messages[0].linear.x == 0.0
    assert publisher.messages[0].angular.z == 0.0


def test_detection_and_id_transitions_invalidate_cached_sample() -> None:
    invalidations = []
    harness = SimpleNamespace(
        _detected=False,
        _selected_tag_id=-1,
        _invalidate_sample=lambda: invalidations.append(True),
    )
    ApproachControllerNode._on_detected(
        harness, SimpleNamespace(data=True)
    )
    ApproachControllerNode._on_tag_id(harness, SimpleNamespace(data=0))
    assert harness._detected is True
    assert harness._selected_tag_id == 0
    assert invalidations == [True, True]
