"""ROS-independent tests for Leader target-pose command calculation."""

from math import cos, inf, nan, radians, sin

import pytest

from leader_approach_control.approach_controller_logic import (
    ALIGNED,
    APPROACH,
    BLIND_FINAL_APPROACH,
    COARSE_TRACK,
    FINAL_APPROACH,
    FINAL_YAW_ALIGN,
    FINE_ALIGN_LEFT,
    FINE_ALIGN_RIGHT,
    NEAR_ALIGN,
    RECENTER,
    STABILIZING,
    TAG_LOST,
    TOO_CLOSE,
    TURN_LEFT,
    TURN_RIGHT,
    BaseControlMeasurement,
    ControllerParameters,
    PlanarCommand,
    compute_approach_command,
    quaternion_yaw,
    sample_is_fresh,
    samples_are_coherent,
)


def make_parameters(**overrides: object) -> ControllerParameters:
    values = {
        "linear_gain": 0.20,
        "angular_gain": 0.80,
        "lateral_gain": 0.50,
        "max_raw_linear_speed": 0.05,
        "max_raw_angular_speed": 0.20,
        "near_max_angular_speed": 0.10,
        "max_final_linear_speed": 0.02,
        "max_final_angular_speed": 0.08,
    }
    values.update(overrides)
    return ControllerParameters(**values)


def compute(
    state: str,
    measurement: BaseControlMeasurement = BaseControlMeasurement(0.10, 0.0, 0.0),
    mode: str = COARSE_TRACK,
    **gate_overrides: bool,
) -> PlanarCommand:
    gates = {
        "enabled": True,
        "detected": True,
        "tag_valid": True,
        "fresh": True,
        "coherent": True,
    }
    gates.update(gate_overrides)
    return compute_approach_command(
        state, mode, measurement, make_parameters(), **gates
    )


def test_turn_left_is_positive_rotation_only() -> None:
    command = compute(TURN_LEFT, BaseControlMeasurement(0.20, 0.05, 0.0))
    assert command.linear_x == 0.0
    assert command.angular_z > 0.0


def test_turn_right_is_negative_rotation_only() -> None:
    command = compute(TURN_RIGHT, BaseControlMeasurement(0.20, -0.05, 0.0))
    assert command.linear_x == 0.0
    assert command.angular_z < 0.0


def test_prealign_approach_is_forward_with_target_bearing_correction() -> None:
    left = compute(APPROACH, BaseControlMeasurement(0.20, 0.01, 0.0))
    right = compute(APPROACH, BaseControlMeasurement(0.20, -0.01, 0.0))
    assert left.linear_x > 0.0 and left.angular_z > 0.0
    assert right.linear_x > 0.0 and right.angular_z < 0.0


def test_final_yaw_align_is_rotation_only() -> None:
    left = compute(
        FINE_ALIGN_LEFT,
        BaseControlMeasurement(0.10, 0.0, 0.10),
        mode=FINAL_YAW_ALIGN,
    )
    right = compute(
        FINE_ALIGN_RIGHT,
        BaseControlMeasurement(0.10, 0.0, -0.10),
        mode=FINAL_YAW_ALIGN,
    )
    assert left.linear_x == 0.0 and left.angular_z > 0.0
    assert right.linear_x == 0.0 and right.angular_z < 0.0

    limited = compute(
        FINE_ALIGN_LEFT,
        BaseControlMeasurement(0.10, 0.0, 1.0),
        mode=FINAL_YAW_ALIGN,
    )
    assert limited.angular_z == pytest.approx(0.08)


def test_final_approach_is_slower_and_uses_small_correction() -> None:
    command = compute(
        FINAL_APPROACH,
        BaseControlMeasurement(1.0, 1.0, 1.0),
        mode=FINAL_APPROACH,
    )
    assert command.linear_x == pytest.approx(0.02)
    assert command.angular_z == pytest.approx(0.08)


def test_final_approach_never_reverses() -> None:
    assert compute(
        FINAL_APPROACH,
        BaseControlMeasurement(-0.01, 0.0, 0.0),
        mode=FINAL_APPROACH,
    ) == PlanarCommand()


def test_blind_final_approach_is_low_speed_forward_only_without_detection() -> None:
    command = compute(
        FINAL_APPROACH,
        BaseControlMeasurement(0.07, 0.0, 0.0),
        mode=BLIND_FINAL_APPROACH,
        detected=False,
        tag_valid=False,
    )
    assert command.linear_x == pytest.approx(0.015)
    assert command.angular_z == 0.0


def test_blind_final_approach_rejects_wrong_state() -> None:
    assert compute(
        TAG_LOST,
        BaseControlMeasurement(0.07, 0.0, 0.0),
        mode=BLIND_FINAL_APPROACH,
        detected=False,
        tag_valid=False,
    ) == PlanarCommand()


@pytest.mark.parametrize("state", [TAG_LOST, ALIGNED, STABILIZING, TOO_CLOSE])
def test_required_stop_states_are_zero(state: str) -> None:
    assert compute(state) == PlanarCommand()


@pytest.mark.parametrize(
    "gate", ["enabled", "detected", "tag_valid", "fresh", "coherent"]
)
def test_failed_safety_gate_is_zero(gate: str) -> None:
    assert compute(APPROACH, **{gate: False}) == PlanarCommand()


@pytest.mark.parametrize(
    "measurement",
    [
        BaseControlMeasurement(nan, 0.0, 0.0),
        BaseControlMeasurement(inf, 0.0, 0.0),
        BaseControlMeasurement(0.10, nan, 0.0),
        BaseControlMeasurement(0.10, 0.0, -inf),
    ],
)
def test_invalid_measurement_is_zero(measurement: BaseControlMeasurement) -> None:
    assert compute(APPROACH, measurement) == PlanarCommand()


def test_raw_prealign_candidates_are_saturated() -> None:
    command = compute(APPROACH, BaseControlMeasurement(10.0, 10.0, 0.0))
    assert command.linear_x == pytest.approx(0.05)
    assert command.angular_z == pytest.approx(0.20)


def test_near_and_recenter_angular_commands_use_near_limit() -> None:
    near = compute(
        APPROACH,
        BaseControlMeasurement(0.01, 1.0, 0.0),
        mode=NEAR_ALIGN,
    )
    recenter = compute(
        TURN_LEFT,
        BaseControlMeasurement(0.01, 1.0, 0.0),
        mode=RECENTER,
    )
    assert near.angular_z == pytest.approx(0.10)
    assert recenter.linear_x == 0.0
    assert recenter.angular_z == pytest.approx(0.10)


def test_state_and_error_sign_mismatch_fails_closed() -> None:
    assert compute(
        TURN_LEFT, BaseControlMeasurement(0.20, -0.03, 0.0)
    ) == PlanarCommand()
    assert compute(
        FINE_ALIGN_RIGHT,
        BaseControlMeasurement(0.10, 0.0, 0.04),
        mode=FINAL_YAW_ALIGN,
    ) == PlanarCommand()


def test_unknown_state_is_zero() -> None:
    assert compute("INVALID") == PlanarCommand()


def test_unknown_or_incompatible_mode_is_zero() -> None:
    assert compute(APPROACH, mode="INVALID") == PlanarCommand()
    assert compute(APPROACH, mode=RECENTER) == PlanarCommand()


def test_planar_target_quaternion_yaw_handles_wraparound() -> None:
    target = radians(179.0)
    quaternion = (0.0, 0.0, sin(target / 2.0), cos(target / 2.0))
    assert quaternion_yaw(quaternion) == pytest.approx(target)
    assert quaternion_yaw((0.0, 0.0, 0.0, 0.0)) is None


def test_source_and_receipt_freshness_include_timeout_boundary() -> None:
    assert sample_is_fresh(10.35, 10.0, 20.35, 20.0, 0.35)
    assert not sample_is_fresh(10.351, 10.0, 20.35, 20.0, 0.35)
    assert not sample_is_fresh(10.35, 10.0, 20.351, 20.0, 0.35)
    assert not sample_is_fresh(9.9, 10.0, 20.0, 20.0, 0.35)


def test_state_must_follow_target_pose_within_sync_window() -> None:
    assert samples_are_coherent(20.0, 20.10, 0.10)
    assert not samples_are_coherent(20.0, 20.101, 0.10)
    assert not samples_are_coherent(20.1, 20.0, 0.10)


@pytest.mark.parametrize(
    "parameters",
    [
        make_parameters(linear_gain=0.0),
        make_parameters(angular_gain=-1.0),
        make_parameters(lateral_gain=-1.0),
        make_parameters(max_raw_linear_speed=0.0),
        make_parameters(max_raw_angular_speed=inf),
        make_parameters(near_max_angular_speed=0.21),
        make_parameters(max_final_linear_speed=0.06),
        make_parameters(max_final_angular_speed=0.21),
    ],
)
def test_invalid_parameters_are_rejected(parameters: ControllerParameters) -> None:
    with pytest.raises(ValueError):
        parameters.validate()
    NEAR_ALIGN,
    RECENTER,
