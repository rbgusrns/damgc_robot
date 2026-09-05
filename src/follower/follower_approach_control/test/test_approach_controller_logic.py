"""Tests for Follower hybrid mode command calculation."""

from math import nan

import pytest

from follower_approach_control.approach_controller_logic import (
    ALIGNED,
    APPROACH,
    BLIND_FINAL_APPROACH,
    COARSE_TRACK,
    FINAL_APPROACH,
    FINAL_YAW_ALIGN,
    FINE_ALIGN_LEFT,
    NEAR_ALIGN,
    RECENTER,
    STABILIZING,
    TAG_LOST,
    TURN_LEFT,
    BaseControlMeasurement,
    ControllerParameters,
    PlanarCommand,
    compute_approach_command,
    quaternion_yaw,
    sample_is_fresh,
)


PARAMS = ControllerParameters(0.2, 0.8, 0.5, 0.05, 0.2, 0.1, 0.02, 0.08, 0.015)


def command(state, mode, sample=BaseControlMeasurement(0.2, 0.0, 0.0), **gates):
    values = dict(enabled=True, detected=True, tag_valid=True, fresh=True, coherent=True)
    values.update(gates)
    return compute_approach_command(state, mode, sample, PARAMS, **values)


def test_coarse_and_near_approach_are_forward_with_bearing_correction() -> None:
    coarse = command(APPROACH, COARSE_TRACK, BaseControlMeasurement(0.5, 0.1, 0.0))
    near = command(APPROACH, NEAR_ALIGN, BaseControlMeasurement(0.2, -0.1, 0.0))
    assert coarse.linear_x > 0.0 and coarse.angular_z > 0.0
    assert near.linear_x > 0.0 and near.angular_z < 0.0
    assert abs(near.angular_z) <= PARAMS.near_max_angular_speed


def test_recenter_is_rotation_only() -> None:
    result = command(TURN_LEFT, RECENTER, BaseControlMeasurement(0.3, 0.2, 0.0))
    assert result.linear_x == 0.0
    assert 0.0 < result.angular_z <= PARAMS.near_max_angular_speed


def test_final_yaw_is_rotation_only() -> None:
    result = command(
        FINE_ALIGN_LEFT, FINAL_YAW_ALIGN, BaseControlMeasurement(0.0, 0.0, 0.2)
    )
    assert result.linear_x == 0.0
    assert 0.0 < result.angular_z <= PARAMS.max_final_angular_speed


def test_final_approach_is_slow_and_never_reverses() -> None:
    result = command(
        FINAL_APPROACH, FINAL_APPROACH, BaseControlMeasurement(0.2, 0.01, 0.02)
    )
    assert 0.0 < result.linear_x <= PARAMS.max_final_linear_speed
    assert abs(result.angular_z) <= PARAMS.max_final_angular_speed
    assert command(
        FINAL_APPROACH, FINAL_APPROACH, BaseControlMeasurement(-0.1, 0.0, 0.0)
    ) == PlanarCommand()


def test_blind_is_forward_only_for_exact_pair_without_detection() -> None:
    result = command(
        FINAL_APPROACH,
        BLIND_FINAL_APPROACH,
        detected=False,
        tag_valid=False,
    )
    assert result == PlanarCommand(linear_x=PARAMS.blind_final_speed)
    assert command(APPROACH, BLIND_FINAL_APPROACH) == PlanarCommand()


@pytest.mark.parametrize("state", [TAG_LOST, STABILIZING, ALIGNED])
def test_required_stop_states_are_zero(state) -> None:
    assert command(state, state) == PlanarCommand()


@pytest.mark.parametrize("gate", ["enabled", "fresh", "coherent"])
def test_failed_safety_gate_is_zero(gate) -> None:
    assert command(APPROACH, COARSE_TRACK, **{gate: False}) == PlanarCommand()


def test_detection_and_tag_are_required_except_blind() -> None:
    assert command(APPROACH, COARSE_TRACK, detected=False) == PlanarCommand()
    assert command(APPROACH, COARSE_TRACK, tag_valid=False) == PlanarCommand()


def test_unknown_incompatible_and_nonfinite_commands_are_zero() -> None:
    assert command("UNKNOWN", COARSE_TRACK) == PlanarCommand()
    assert command(APPROACH, FINAL_YAW_ALIGN) == PlanarCommand()
    assert command(
        APPROACH, COARSE_TRACK, BaseControlMeasurement(nan, 0.0, 0.0)
    ) == PlanarCommand()


def test_quaternion_yaw_and_source_receipt_freshness() -> None:
    assert quaternion_yaw((0.0, 0.0, 0.0, 1.0)) == pytest.approx(0.0)
    assert quaternion_yaw((0.0, 0.0, 0.0, 0.0)) is None
    assert sample_is_fresh(10.2, 10.0, 20.2, 20.0, 0.2)
    assert not sample_is_fresh(10.3, 10.0, 20.2, 20.0, 0.2)
    assert not sample_is_fresh(10.2, 10.0, 20.3, 20.0, 0.2)


@pytest.mark.parametrize(
    "invalid",
    [
        ControllerParameters(0.0, 0.8, 0.5, 0.05, 0.2, 0.1, 0.02, 0.08, 0.015),
        ControllerParameters(0.2, 0.8, 0.5, 0.05, 0.2, 0.3, 0.02, 0.08, 0.015),
        ControllerParameters(0.2, 0.8, 0.5, 0.05, 0.2, 0.1, 0.06, 0.08, 0.015),
        ControllerParameters(0.2, 0.8, 0.5, 0.05, 0.2, 0.1, 0.02, 0.08, 0.03),
    ],
)
def test_invalid_parameters_are_rejected(invalid) -> None:
    with pytest.raises(ValueError):
        invalid.validate()
