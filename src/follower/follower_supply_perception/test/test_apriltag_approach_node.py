"""Focused tests for receipt timing, atomic commands, and blind-final safety."""

from math import radians
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time

from follower_supply_perception.approach_logic import ApproachState, TagObservation
import follower_supply_perception.apriltag_approach_node as node_module
from follower_supply_perception.apriltag_approach_node import AprilTagApproachNode
from follower_supply_perception.base_alignment_logic import AlignmentDecision, ControlMode


class RecordingPublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def test_source_stamp_identifies_new_samples_but_receipt_time_drives_age() -> None:
    harness = SimpleNamespace(
        _last_processed_observation_stamps={}, _last_observation_receipts={}
    )
    sample = TagObservation(0, 0.0, 0.0, 1.0, (0.0, 0.0, 0.0, 1.0), 100)

    assert AprilTagApproachNode._accept_observation_stamp(harness, sample, 50.0)
    assert harness._last_observation_receipts[0] == 50.0
    assert not AprilTagApproachNode._accept_observation_stamp(harness, sample, 60.0)
    assert harness._last_observation_receipts[0] == 50.0

    newer = TagObservation(0, 0.0, 0.0, 1.0, sample.quaternion, 101)
    assert AprilTagApproachNode._accept_observation_stamp(harness, newer, 60.0)
    assert harness._last_observation_receipts[0] == 60.0


def test_atomic_command_keeps_pose_mode_and_state_in_one_generation() -> None:
    publisher = RecordingPublisher()
    harness = SimpleNamespace(
        _base_frame="base_link",
        _command_pub=publisher,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=20.0)),
    )
    first_pose = PoseStamped()
    first_pose.header.frame_id = "base_link"
    first_pose.header.stamp = Time(seconds=10.0).to_msg()
    first_pose.pose.position.x = 0.3
    first = AlignmentDecision(
        ApproachState.FINAL_APPROACH, ControlMode.FINAL_APPROACH
    )
    AprilTagApproachNode._publish_atomic_command(harness, first_pose, first)

    second = AlignmentDecision(ApproachState.TAG_LOST, ControlMode.TAG_LOST)
    AprilTagApproachNode._publish_atomic_command(harness, None, second)

    assert publisher.messages[0].header == first_pose.header
    assert publisher.messages[0].target_pose.position.x == pytest.approx(0.3)
    assert publisher.messages[0].control_mode == "FINAL_APPROACH"
    assert publisher.messages[0].alignment_state == "FINAL_APPROACH"
    assert publisher.messages[1].target_pose.position.x == 0.0
    assert publisher.messages[1].control_mode == "TAG_LOST"
    assert publisher.messages[1].alignment_state == "TAG_LOST"


def test_odom_requires_fresh_source_and_local_receipt_times() -> None:
    harness = SimpleNamespace(
        _last_odom=(1.0, 2.0, 0.1, 10.0, 100.0),
        _blind_odom_timeout=0.25,
    )
    assert AprilTagApproachNode._fresh_odom(harness, 10.2, 100.2) == (
        1.0,
        2.0,
        0.1,
    )
    assert AprilTagApproachNode._fresh_odom(harness, 10.3, 100.2) is None
    assert AprilTagApproachNode._fresh_odom(harness, 10.2, 100.3) is None
    harness._last_odom = None
    assert AprilTagApproachNode._fresh_odom(harness, 10.0, 100.0) is None


def blind_cycle_harness(current_odom) -> SimpleNamespace:
    commands = []
    losses = []
    harness = SimpleNamespace(
        _blind_active=True,
        _blind_completed=False,
        _blind_start_receipt=10.0,
        _blind_max_duration=5.0,
        _blind_start_odom=(0.0, 0.0, 0.0),
        _blind_previous_odom=(0.0, 0.0, 0.0),
        _blind_reverse_tolerance=0.01,
        _blind_max_odom_step=0.05,
        _blind_max_lateral_deviation=0.03,
        _blind_max_yaw_deviation_deg=12.0,
        _blind_max_distance=0.10,
        _blind_planned_distance=0.08,
        _last_odom_progress=0.0,
        _last_valid_tag_x=0.33,
        _last_valid_receipt=9.9,
        _last_valid_yaw_error=0.0,
        _last_valid_cross_track=0.0,
        _fresh_odom=lambda *_args: current_odom,
        _publish_blind_command=lambda state, mode: commands.append((state, mode)),
        _publish_base_lost=lambda *_args: losses.append(True),
    )
    harness._clear_blind_plan = lambda: AprilTagApproachNode._clear_blind_plan(
        harness
    )
    harness._clear_final_sample = lambda: AprilTagApproachNode._clear_final_sample(
        harness
    )
    harness._abort_blind = lambda *args: AprilTagApproachNode._abort_blind(
        harness, *args
    )
    harness.commands = commands
    harness.losses = losses
    return harness


@pytest.mark.parametrize(
    "current_odom",
    [
        None,
        (0.06, 0.0, 0.0),  # odometry jump
        (0.0, 0.04, 0.0),  # lateral deviation
        (0.0, 0.0, radians(13.0)),  # yaw deviation
        (-0.02, 0.0, 0.0),  # reverse motion
    ],
)
def test_blind_safety_violation_aborts_to_loss(current_odom) -> None:
    harness = blind_cycle_harness(current_odom)
    AprilTagApproachNode._publish_blind_cycle(harness, 20.0, 10.1)
    assert harness.losses == [True]
    assert not harness._blind_active
    assert harness.commands == []


def test_blind_timeout_aborts() -> None:
    harness = blind_cycle_harness((0.0, 0.0, 0.0))
    AprilTagApproachNode._publish_blind_cycle(harness, 20.0, 15.1)
    assert harness.losses == [True]


def test_blind_total_distance_limit_aborts_independently_of_step_limit() -> None:
    harness = blind_cycle_harness((0.11, 0.0, 0.0))
    harness._blind_previous_odom = (0.08, 0.0, 0.0)
    AprilTagApproachNode._publish_blind_cycle(harness, 20.0, 10.1)
    assert harness.losses == [True]


def test_blind_progress_continues_then_completes_aligned() -> None:
    harness = blind_cycle_harness((0.02, 0.0, 0.0))
    AprilTagApproachNode._publish_blind_cycle(harness, 20.0, 10.1)
    assert harness.commands[-1] == (
        ApproachState.FINAL_APPROACH,
        ControlMode.BLIND_FINAL_APPROACH,
    )
    assert harness._last_odom_progress == pytest.approx(0.02)

    harness._blind_previous_odom = (0.04, 0.0, 0.0)
    harness._fresh_odom = lambda *_args: (0.08, 0.0, 0.0)
    AprilTagApproachNode._publish_blind_cycle(harness, 20.1, 10.2)
    assert harness.commands[-1] == (ApproachState.ALIGNED, ControlMode.ALIGNED)
    assert harness._blind_completed
    assert not harness._blind_active


def test_odom_yaw_projection_uses_start_heading() -> None:
    harness = blind_cycle_harness((0.0, 0.02, radians(90.0)))
    harness._blind_start_odom = (0.0, 0.0, radians(90.0))
    harness._blind_previous_odom = harness._blind_start_odom
    harness._blind_planned_distance = 0.02
    AprilTagApproachNode._publish_blind_cycle(harness, 20.0, 10.1)
    assert harness.commands[-1] == (ApproachState.ALIGNED, ControlMode.ALIGNED)


def test_fresh_visual_sample_cancels_blind_and_resumes_visual(monkeypatch) -> None:
    sample = TagObservation(
        0, 0.0, 0.0, 0.3, (0.0, 0.0, 0.0, 1.0), 19_500_000_000
    )
    published = []
    cleared = []
    harness = SimpleNamespace(
        _blind_completed=False,
        _blind_active=True,
        _selection_mode="priority",
        _active_tag_id=0,
        _tag_receipt_timeout=0.35,
        _blind_last_tag_max_age=0.25,
        _last_valid_receipt=19.0,
        _last_processed_observation_stamps={},
        _last_observation_receipts={},
        _candidate_ids=lambda: (0,),
        _lookup_observation=lambda *_args: sample,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=20.0)),
        _translation_filter=SimpleNamespace(add=lambda *_args: "measurement"),
        _normal_filter=SimpleNamespace(reset=lambda: None),
        _state_machine=SimpleNamespace(
            update=lambda *_args: ApproachState.APPROACH, reset=lambda: None
        ),
        _base_state_machine=SimpleNamespace(reset=lambda: None),
        _publish_valid=lambda *args: published.append(args),
    )

    def clear_blind() -> None:
        harness._blind_active = False
        cleared.append(True)

    harness._clear_blind_plan = clear_blind
    harness._accept_observation_stamp = lambda observation, receipt: (
        AprilTagApproachNode._accept_observation_stamp(harness, observation, receipt)
    )
    monkeypatch.setattr(node_module.time, "monotonic", lambda: 100.0)

    AprilTagApproachNode._on_timer(harness)

    assert cleared == [True]
    assert len(published) == 1
    assert published[0][-1] is True
