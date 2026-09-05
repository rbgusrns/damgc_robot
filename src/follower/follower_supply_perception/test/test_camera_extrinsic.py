"""Tests for the repository-owned Follower camera static TF chain."""

from math import pi

import pytest

from follower_supply_perception.camera_extrinsic import (
    BASE_FRAME,
    BASE_TO_CAMERA_BODY,
    CAMERA_BODY_FRAME,
    CAMERA_BODY_TO_OPTICAL,
    CAMERA_OPTICAL_FRAME,
    FOLLOWER_CAMERA_TRANSFORMS,
    FixedTransform,
    validate_transform_chain,
)


def test_measured_mounting_translation_is_in_base_to_body_transform() -> None:
    assert BASE_TO_CAMERA_BODY.parent_frame == BASE_FRAME == "base_link"
    assert BASE_TO_CAMERA_BODY.child_frame == CAMERA_BODY_FRAME
    assert BASE_TO_CAMERA_BODY.xyz == pytest.approx((0.042, 0.01, 0.120))
    assert BASE_TO_CAMERA_BODY.rpy == pytest.approx((0.0, 0.0, 0.0))


def test_optical_rotation_is_separate_from_mounting_translation() -> None:
    assert CAMERA_BODY_TO_OPTICAL.parent_frame == CAMERA_BODY_FRAME
    assert CAMERA_BODY_TO_OPTICAL.child_frame == CAMERA_OPTICAL_FRAME
    assert CAMERA_OPTICAL_FRAME == "follower/follower_camera_optical_frame"
    assert CAMERA_BODY_TO_OPTICAL.xyz == pytest.approx((0.0, 0.0, 0.0))
    assert CAMERA_BODY_TO_OPTICAL.rpy == pytest.approx((-pi / 2.0, 0.0, -pi / 2.0))


def test_transform_chain_has_one_parent_per_child() -> None:
    validate_transform_chain(FOLLOWER_CAMERA_TRANSFORMS)
    children = [transform.child_frame for transform in FOLLOWER_CAMERA_TRANSFORMS]
    assert len(children) == len(set(children))
    assert CAMERA_BODY_TO_OPTICAL.parent_frame == BASE_TO_CAMERA_BODY.child_frame


def test_static_publisher_arguments_use_explicit_humble_flags() -> None:
    arguments = BASE_TO_CAMERA_BODY.publisher_arguments()
    assert arguments[-4:] == (
        "--frame-id",
        BASE_FRAME,
        "--child-frame-id",
        CAMERA_BODY_FRAME,
    )


def test_duplicate_child_frame_is_rejected() -> None:
    duplicate = FixedTransform(
        parent_frame="another_parent",
        child_frame=CAMERA_BODY_FRAME,
        xyz=(0.0, 0.0, 0.0),
        rpy=(0.0, 0.0, 0.0),
    )
    with pytest.raises(ValueError, match="exactly one publisher"):
        validate_transform_chain((BASE_TO_CAMERA_BODY, duplicate))


def test_disconnected_transform_chain_is_rejected() -> None:
    disconnected = FixedTransform(
        parent_frame="wrong_parent",
        child_frame="child",
        xyz=(0.0, 0.0, 0.0),
        rpy=(0.0, 0.0, 0.0),
    )
    with pytest.raises(ValueError, match="ordered chain"):
        validate_transform_chain((BASE_TO_CAMERA_BODY, disconnected))
