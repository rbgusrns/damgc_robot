"""Tests for ROS-independent camera XYZ projection logic."""

import math

import pytest

from rescue_robot_survivor.geometry_logic import (
    CameraIntrinsics,
    deproject_pixel_to_camera_xyz,
    get_rectified_intrinsics,
    roi_center_pixel,
)


P = [600.0, 0.0, 320.0, 0.0, 0.0, 600.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]


def intrinsics():
    return CameraIntrinsics(600.0, 600.0, 320.0, 240.0, 640, 480)


def test_extracts_rectified_intrinsics_from_projection_matrix():
    result = get_rectified_intrinsics(P, 640, 480, 640, 480)
    assert result == intrinsics()


@pytest.mark.parametrize(
    "projection",
    [
        P[:11],
        [0.0 if index == 0 else value for index, value in enumerate(P)],
        [0.0 if index == 5 else value for index, value in enumerate(P)],
        [math.nan if index == 2 else value for index, value in enumerate(P)],
        [math.inf if index == 6 else value for index, value in enumerate(P)],
    ],
)
def test_rejects_invalid_projection_intrinsics(projection):
    assert get_rectified_intrinsics(projection, 640, 480, 640, 480) is None


def test_rejects_camera_info_resolution_mismatch():
    assert get_rectified_intrinsics(P, 640, 480, 1280, 720) is None


def test_calculates_half_open_roi_center_and_rejects_empty_roi():
    assert roi_center_pixel((10, 20, 20, 30)) == (14.5, 24.5)
    assert roi_center_pixel((4, 4, 5, 5)) == (4.0, 4.0)
    assert roi_center_pixel((4, 4, 4, 5)) is None


def test_center_pixel_has_zero_x_and_y_and_preserves_z():
    point = deproject_pixel_to_camera_xyz(320.0, 240.0, 2.5, intrinsics())
    assert point.x == pytest.approx(0.0)
    assert point.y == pytest.approx(0.0)
    assert point.z == pytest.approx(2.5)


def test_optical_frame_axis_signs():
    left = deproject_pixel_to_camera_xyz(200.0, 240.0, 2.0, intrinsics())
    right = deproject_pixel_to_camera_xyz(440.0, 240.0, 2.0, intrinsics())
    above = deproject_pixel_to_camera_xyz(320.0, 120.0, 2.0, intrinsics())
    below = deproject_pixel_to_camera_xyz(320.0, 360.0, 2.0, intrinsics())
    assert left.x < 0.0
    assert right.x > 0.0
    assert above.y < 0.0
    assert below.y > 0.0


@pytest.mark.parametrize(
    "u,v,z",
    [
        (320.0, 240.0, 0.0),
        (320.0, 240.0, -1.0),
        (math.nan, 240.0, 1.0),
        (320.0, math.inf, 1.0),
        (320.0, 240.0, math.nan),
        (-0.1, 240.0, 1.0),
        (640.0, 240.0, 1.0),
        (320.0, 480.0, 1.0),
    ],
)
def test_rejects_invalid_pixel_or_depth(u, v, z):
    assert deproject_pixel_to_camera_xyz(u, v, z, intrinsics()) is None


def test_multiple_pixels_produce_independent_points():
    points = [
        deproject_pixel_to_camera_xyz(200.0, 240.0, 1.0, intrinsics()),
        deproject_pixel_to_camera_xyz(440.0, 240.0, 3.0, intrinsics()),
    ]
    assert points[0].x == pytest.approx(-0.2)
    assert points[0].z == pytest.approx(1.0)
    assert points[1].x == pytest.approx(0.6)
    assert points[1].z == pytest.approx(3.0)
