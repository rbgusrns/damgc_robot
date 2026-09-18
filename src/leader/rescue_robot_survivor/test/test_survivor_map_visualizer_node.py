"""Current-candidate RViz marker contracts and stale cleanup."""

from math import inf, nan
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker

from rescue_robot_survivor.survivor_map_visualizer_node import (
    SurvivorMapVisualizerNode,
    build_marker_array,
)


def make_positions(points):
    """Create map positions with one shared RGB image timestamp."""
    message = PoseArray()
    message.header.frame_id = "map"
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    for x, y, z in points:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def build(message, previous_indices=frozenset()):
    """Use the Phase A marker defaults."""
    return build_marker_array(
        message, previous_indices, "survivor_current", 2.0, 0.20, 0.18, 0.30
    )


def test_one_candidate_makes_stamped_position_and_text_markers():
    message = make_positions([(2.53, -0.06, 0.51)])

    array, indices, skipped = build(message)

    assert indices == {0}
    assert skipped == 0
    assert len(array.markers) == 2
    sphere, label = array.markers
    assert [(m.ns, m.id, m.action) for m in array.markers] == [
        ("survivor_current", 0, Marker.ADD),
        ("survivor_current", 1, Marker.ADD),
    ]
    assert sphere.type == Marker.SPHERE
    assert label.type == Marker.TEXT_VIEW_FACING
    assert all(m.header.frame_id == "map" for m in array.markers)
    assert all(m.header.stamp == message.header.stamp for m in array.markers)
    assert (sphere.pose.position.x, sphere.pose.position.y,
            sphere.pose.position.z) == (2.53, -0.06, 0.51)
    assert (label.pose.position.x, label.pose.position.y,
            label.pose.position.z) == (2.53, -0.06, 0.81)
    assert "Survivor candidate 1" in label.text
    assert "X=2.53 Y=-0.06 Z=0.51" in label.text
    assert all((m.pose.orientation.x, m.pose.orientation.y,
                m.pose.orientation.z, m.pose.orientation.w) ==
               (0.0, 0.0, 0.0, 1.0) for m in array.markers)
    assert sphere.scale.x == sphere.scale.y == sphere.scale.z == 0.20
    assert label.scale.z == 0.18
    assert all(m.color.a > 0.0 for m in array.markers)
    assert all((m.lifetime.sec, m.lifetime.nanosec) == (2, 0)
               for m in array.markers)


def test_multiple_candidates_have_frame_local_ids():
    array, indices, skipped = build(make_positions([
        (1.0, 0.0, 0.5), (2.0, 0.0, 0.5), (3.0, 0.0, 0.5),
    ]))

    assert len(array.markers) == 6
    assert [m.id for m in array.markers] == [0, 1, 2, 3, 4, 5]
    assert indices == {0, 1, 2}
    assert skipped == 0
    assert "Survivor candidate 3" in array.markers[-1].text


def test_invalid_position_skips_only_its_candidate():
    for value in (nan, inf, -inf):
        array, indices, skipped = build(make_positions([
            (1.0, 0.0, 0.5), (value, 1.0, 0.5), (3.0, 0.0, 0.5),
        ]))
        assert [m.id for m in array.markers] == [0, 1, 4, 5]
        assert indices == {0, 2}
        assert skipped == 1
        assert "Survivor candidate 3" in array.markers[-1].text


def test_removed_indices_get_both_delete_actions():
    array, indices, skipped = build(
        make_positions([(1.0, 0.0, 0.5)]), {0, 1, 2}
    )

    assert [m.id for m in array.markers] == [0, 1, 2, 3, 4, 5]
    assert [m.action for m in array.markers] == [
        Marker.ADD, Marker.ADD, Marker.DELETE, Marker.DELETE,
        Marker.DELETE, Marker.DELETE,
    ]
    assert all(m.ns == "survivor_current" for m in array.markers)
    assert all(m.header.frame_id == "map" for m in array.markers)
    assert indices == {0}
    assert skipped == 0


def test_empty_input_deletes_every_previous_marker():
    array, indices, skipped = build(make_positions([]), {0, 2})

    assert [m.id for m in array.markers] == [0, 1, 4, 5]
    assert all(m.action == Marker.DELETE for m in array.markers)
    assert indices == set()
    assert skipped == 0


def test_callback_updates_active_indices_and_skips_empty_frame():
    harness = SimpleNamespace(
        _active_indices={0, 1},
        _marker_namespace="survivor_current",
        _marker_lifetime_sec=2.0,
        _marker_scale=0.20,
        _text_height=0.18,
        _text_z_offset=0.30,
        _publisher=Mock(),
        get_logger=Mock(),
    )
    message = make_positions([(1.0, 0.0, 0.5)])
    SurvivorMapVisualizerNode._positions_callback(harness, message)
    assert harness._active_indices == {0}
    published = harness._publisher.publish.call_args.args[0]
    assert [m.id for m in published.markers if m.action == Marker.DELETE] == [
        2, 3,
    ]

    empty = make_positions([])
    SurvivorMapVisualizerNode._positions_callback(harness, empty)
    assert harness._active_indices == set()
    empty_markers = harness._publisher.publish.call_args.args[0].markers
    assert [m.id for m in empty_markers] == [
        0, 1,
    ]

    malformed = make_positions([(1.0, 0.0, 0.5)])
    malformed.header.frame_id = ""
    harness._publisher.publish.reset_mock()
    SurvivorMapVisualizerNode._positions_callback(harness, malformed)
    harness._publisher.publish.assert_not_called()
    harness.get_logger().warning.assert_called_once()


def test_invalid_parameters_are_rejected():
    base = dict(
        _input_topic="/leader/survivor/map_positions",
        _output_topic="/leader/survivor/map_markers",
        _marker_namespace="survivor_current",
        _marker_lifetime_sec=2.0,
        _marker_scale=0.20,
        _text_height=0.18,
        _text_z_offset=0.30,
    )
    for field, value in (
        ("_marker_lifetime_sec", 0.0),
        ("_marker_lifetime_sec", inf),
        ("_marker_scale", nan),
        ("_text_height", -1.0),
        ("_text_z_offset", -0.1),
        ("_marker_namespace", ""),
    ):
        params = base.copy()
        params[field] = value
        try:
            SurvivorMapVisualizerNode._validate_parameters(
                SimpleNamespace(**params)
            )
        except ValueError:
            pass
        else:
            raise AssertionError(f"{field}={value} should be rejected")
