"""Typed ROS wrapper contracts for the survivor registry."""

from math import inf
import threading
from types import SimpleNamespace
from unittest.mock import Mock

from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import Pose, PoseArray
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from rescue_robot_interfaces.msg import SurvivorTrack
from std_srvs.srv import Trigger

from rescue_robot_survivor.survivor_registry_core import (
    Position3D,
    RegistryConfig,
    SurvivorRegistry,
    TrackSnapshot,
    TrackStatus,
)
from rescue_robot_survivor.survivor_registry_node import (
    POSITION_QOS,
    REGISTRY_QOS,
    SurvivorRegistryNode,
    build_track_array,
)


def make_positions(points, frame_id="map", sec=10):
    """Create one map PoseArray."""
    message = PoseArray()
    message.header.frame_id = frame_id
    message.header.stamp.sec = sec
    for x, y, z in points:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def test_build_track_array_populates_every_typed_field():
    snapshot = TrackSnapshot(
        survivor_id=7,
        raw_position=Position3D(1.0, 2.0, 3.0),
        filtered_position=Position3D(1.1, 2.1, 3.1),
        status=TrackStatus.LOST,
        visible=False,
        observation_count=9,
        first_seen_ns=2_000_000_003,
        last_seen_ns=4_000_000_005,
    )
    stamp = TimeMessage(sec=20, nanosec=30)

    message = build_track_array((snapshot,), "map", stamp)

    assert message.header.frame_id == "map"
    assert message.header.stamp == stamp
    assert len(message.tracks) == 1
    track = message.tracks[0]
    assert isinstance(track, SurvivorTrack)
    assert track.id == 7
    assert (track.raw_position.x, track.raw_position.y,
            track.raw_position.z) == (1.0, 2.0, 3.0)
    assert (track.filtered_position.x, track.filtered_position.y,
            track.filtered_position.z) == (1.1, 2.1, 3.1)
    assert track.status == SurvivorTrack.STATUS_LOST
    assert not track.visible
    assert track.observation_count == 9
    assert (track.first_seen.sec, track.first_seen.nanosec) == (2, 3)
    assert (track.last_seen.sec, track.last_seen.nanosec) == (4, 5)


def make_harness(confirm_hits=1):
    """Create a callback harness without starting a ROS graph."""
    now = Time(nanoseconds=10_100_000_000)
    return SimpleNamespace(
        _map_frame="map",
        _registry=SurvivorRegistry(RegistryConfig(
            confirm_hits=confirm_hits
        )),
        _registry_lock=threading.Lock(),
        _publisher=Mock(),
        get_clock=Mock(return_value=SimpleNamespace(
            now=Mock(return_value=now)
        )),
        _warn=Mock(),
    )


def test_callback_skips_bad_header_frame_and_timestamp_without_crash():
    messages = [
        make_positions([(1.0, 0.0, 0.5)], frame_id=""),
        make_positions([(1.0, 0.0, 0.5)], frame_id="odom"),
        make_positions([(1.0, 0.0, 0.5)], sec=0),
    ]
    for message in messages:
        harness = make_harness()
        SurvivorRegistryNode._positions_callback(harness, message)
        harness._publisher.publish.assert_not_called()
        harness._warn.assert_called_once()


def test_callback_skips_non_finite_pose_and_publishes_valid_track():
    harness = make_harness()
    message = make_positions([(1.0, 2.0, 3.0), (inf, 0.0, 0.5)])

    SurvivorRegistryNode._positions_callback(harness, message)

    published = harness._publisher.publish.call_args.args[0]
    assert len(published.tracks) == 1
    assert published.tracks[0].id == 1
    assert published.tracks[0].raw_position.x == 1.0
    harness._warn.assert_called_once()


def test_timer_marks_lost_and_keeps_publishing_snapshot():
    harness = make_harness()
    harness._registry.update((position := Position3D(1.0, 0.0, 0.5),),
                             7_000_000_000)
    assert position.x == 1.0

    SurvivorRegistryNode._timer_callback(harness)

    track = harness._publisher.publish.call_args.args[0].tracks[0]
    assert track.id == 1
    assert track.status == SurvivorTrack.STATUS_LOST
    assert not track.visible


def test_reset_publishes_empty_snapshot_and_restarts_id():
    harness = make_harness()
    harness._registry.update((Position3D(0.0, 0.0, 0.5),),
                             1_000_000_000)
    response = Trigger.Response()

    returned = SurvivorRegistryNode._reset_callback(
        harness, Trigger.Request(), response
    )

    assert returned.success
    assert "next ID is 1" in returned.message
    published = harness._publisher.publish.call_args.args[0]
    assert published.header.frame_id == "map"
    assert published.tracks == []
    assert harness._registry.next_survivor_id == 1


def test_qos_matches_upstream_and_late_subscriber_contract():
    assert POSITION_QOS.depth == 1
    assert POSITION_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert POSITION_QOS.durability == DurabilityPolicy.VOLATILE
    assert REGISTRY_QOS.depth == 1
    assert REGISTRY_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert REGISTRY_QOS.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_invalid_ros_parameters_are_rejected():
    valid = dict(
        _input_topic="/leader/survivor/map_positions",
        _tracks_topic="/leader/survivor/tracks",
        _reset_service_name="/leader/survivor/registry/reset",
        _map_frame="map",
        _registry_publish_hz=2.0,
    )
    for field, value in (
        ("_input_topic", ""),
        ("_tracks_topic", ""),
        ("_reset_service_name", ""),
        ("_map_frame", ""),
        ("_registry_publish_hz", 0.0),
        ("_registry_publish_hz", inf),
    ):
        values = valid.copy()
        values[field] = value
        try:
            SurvivorRegistryNode._validate_ros_parameters(
                SimpleNamespace(**values)
            )
        except ValueError:
            pass
        else:
            raise AssertionError(f"{field}={value} should be rejected")

    values = valid.copy()
    values["_tracks_topic"] = values["_input_topic"]
    try:
        SurvivorRegistryNode._validate_ros_parameters(
            SimpleNamespace(**values)
        )
    except ValueError:
        pass
    else:
        raise AssertionError("input and output topics must differ")
