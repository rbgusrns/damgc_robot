"""Tests for ROS-independent survivor display logic."""

import math

import numpy as np
import pytest

from rescue_robot_survivor.detection_logic import (
    Detection,
    draw_person_detections,
    prepare_person_detections,
)


def detection(center_x, confidence=0.9, class_id=0):
    return Detection(
        center_x - 10, 20, center_x + 10, 80, confidence, class_id
    )


def test_orders_multiple_people_left_to_right():
    people = prepare_person_detections(
        [detection(500), detection(100), detection(300)], 0.5, 640, 480
    )
    assert [person.center_x for person in people] == [100, 300, 500]


def test_keeps_all_people_at_or_above_threshold():
    people = prepare_person_detections(
        [detection(100, 0.49), detection(200, 0.5), detection(300, 0.99)],
        0.5,
        640,
        480,
    )
    assert [person.center_x for person in people] == [200, 300]


def test_filters_non_person_classes():
    people = prepare_person_detections(
        [detection(100, class_id=56), detection(200, class_id=0)],
        0.5,
        640,
        480,
    )
    assert [person.center_x for person in people] == [200]


def test_no_detections_returns_no_people():
    assert prepare_person_detections([], 0.5, 640, 480) == []


def test_clamps_boxes_and_rejects_invalid_values():
    people = prepare_person_detections(
        [
            Detection(-20, -10, 700, 500, 0.9, 0),
            Detection(20, 20, 10, 30, 0.9, 0),
            Detection(math.nan, 0, 10, 10, 0.9, 0),
        ],
        0.5,
        640,
        480,
    )
    assert len(people) == 1
    assert (people[0].x1, people[0].y1, people[0].x2, people[0].y2) == (
        0,
        0,
        639,
        479,
    )


def test_tie_breaker_is_deterministic():
    people = prepare_person_detections(
        [
            Detection(90, 200, 110, 260, 0.8, 0),
            Detection(80, 20, 120, 80, 0.9, 0),
        ],
        0.5,
        640,
        480,
    )
    assert [person.y1 for person in people] == [20, 200]


def test_draws_each_person_and_leaves_empty_frame_unchanged():
    empty = np.zeros((120, 200, 3), dtype=np.uint8)
    unchanged = empty.copy()
    draw_person_detections(unchanged, [])
    assert np.array_equal(empty, unchanged)

    people = prepare_person_detections(
        [detection(50), detection(150)], 0.5, 200, 120
    )
    annotated = empty.copy()
    draw_person_detections(annotated, people)
    assert np.count_nonzero(annotated) > 0


def test_draws_xyz_or_na_near_image_edges_without_failure():
    image = np.zeros((120, 200, 3), dtype=np.uint8)
    people = prepare_person_detections(
        [Detection(0, 0, 40, 60, 0.9, 0), Detection(170, 70, 199, 119, 0.8, 0)],
        0.5,
        200,
        120,
    )

    draw_person_detections(
        image,
        people,
        distances={1: 1.5, 2: None},
        camera_points={1: (-0.2, -0.1, 1.5), 2: None},
        show_camera_xyz=True,
    )

    assert np.count_nonzero(image) > 0


@pytest.mark.parametrize("threshold", [-0.01, 1.01])
def test_rejects_invalid_threshold(threshold):
    with pytest.raises(ValueError):
        prepare_person_detections([], threshold, 640, 480)
