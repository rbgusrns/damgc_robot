"""Tests for ROS-independent aligned depth distance logic."""

import math

import numpy as np
import pytest

from rescue_robot_survivor.depth_logic import (
    calculate_depth_roi,
    convert_depth_to_meters,
    estimate_person_distance,
    extract_valid_depth_values,
    median_depth,
)


def test_calculates_center_roi_and_clamps_to_image():
    assert calculate_depth_roi(0, 0, 20, 20, 0.25, 0.25, 10, 10) == (
        4,
        4,
        6,
        6,
    )
    assert calculate_depth_roi(-20, -10, 8, 8, 0.25, 0.25, 10, 10) == (
        3,
        3,
        5,
        5,
    )


def test_rejects_empty_roi_and_invalid_ratios():
    assert calculate_depth_roi(4, 4, 4, 8, 0.25, 0.25, 10, 10) is None
    with pytest.raises(ValueError):
        calculate_depth_roi(0, 0, 5, 5, 0.0, 0.25, 10, 10)


def test_filters_zero_nan_inf_and_out_of_range_values():
    values = extract_valid_depth_values(
        np.array([[0, 1000, 1010, 1020, 5000]], dtype=np.uint16),
        "16UC1",
        0.001,
        0.2,
        3.0,
    )
    assert np.allclose(values, [1.0, 1.01, 1.02])

    float_values = extract_valid_depth_values(
        np.array([[math.nan, math.inf, 1.0, 0.1, 7.0]], dtype=np.float32),
        "32FC1",
        1.0,
        0.2,
        6.0,
    )
    assert np.allclose(float_values, [1.0])


def test_converts_integer_depth_using_configured_scale():
    converted = convert_depth_to_meters(
        np.array([[1523]], dtype=np.uint16), "16UC1", 0.001
    )
    assert converted[0, 0] == pytest.approx(1.523)


def test_median_requires_minimum_valid_pixel_count():
    result = median_depth(np.array([1.79, 1.80, 1.81, 1.82, 4.50]), 5)
    assert result.distance_m == pytest.approx(1.81)
    assert result.valid_pixel_count == 5
    assert median_depth(np.array([1.0, 1.1]), 3).distance_m is None


def test_estimates_each_bbox_independently_and_handles_edge_bbox():
    depth = np.full((20, 40), 2000, dtype=np.uint16)
    depth[:, :20] = 1000
    depth[:, 20:] = 3000
    first, first_roi = estimate_person_distance(
        depth, "16UC1", (0, 0, 16, 20), 0.25, 0.25, 0.001, 0.2, 6.0, 1
    )
    second, second_roi = estimate_person_distance(
        depth, "16UC1", (24, 0, 40, 20), 0.25, 0.25, 0.001, 0.2, 6.0, 1
    )
    assert first.distance_m == pytest.approx(1.0)
    assert second.distance_m == pytest.approx(3.0)
    assert first_roi is not None and second_roi is not None


def test_unsupported_encoding_is_invalid():
    estimate, roi = estimate_person_distance(
        np.ones((10, 10), dtype=np.uint8),
        "8UC1",
        (0, 0, 10, 10),
        0.25,
        0.25,
        0.001,
        0.2,
        6.0,
        1,
    )
    assert estimate.distance_m is None
    assert roi is not None
