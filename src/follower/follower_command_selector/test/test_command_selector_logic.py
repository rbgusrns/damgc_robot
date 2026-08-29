"""Tests for deterministic source ownership and freshness logic."""

from math import inf, nan

import pytest

from follower_command_selector.command_selector_logic import (
    CommandSource,
    PlanarCommand,
    SelectorParameters,
    command_is_fresh,
    sanitize_command,
    select_command,
)


def parameters(**overrides: float) -> SelectorParameters:
    """Create standard selector timing parameters."""
    values = {
        "approach_timeout": 0.35,
        "cooperation_timeout": 0.50,
        "axis_epsilon": 1.0e-9,
    }
    values.update(overrides)
    return SelectorParameters(**values)


def select(
    source: CommandSource,
    *,
    now: float = 10.0,
    approach: PlanarCommand = PlanarCommand(0.1, 0.2),
    approach_received: float = 9.9,
    cooperation: PlanarCommand = PlanarCommand(0.3, -0.4),
    cooperation_received: float = 9.9,
) -> PlanarCommand:
    """Select with both source caches populated unless overridden."""
    return select_command(
        source,
        now,
        parameters(),
        approach_command=approach,
        approach_received_seconds=approach_received,
        cooperation_command=cooperation,
        cooperation_received_seconds=cooperation_received,
    )


def test_startup_stop_is_zero_even_with_both_sources_active() -> None:
    assert select(CommandSource.STOP) == PlanarCommand()


def test_fresh_approach_selection_uses_only_approach() -> None:
    assert select(CommandSource.APPROACH) == PlanarCommand(0.1, 0.2)


def test_fresh_cooperation_selection_uses_only_cooperation() -> None:
    assert select(CommandSource.COOPERATION) == PlanarCommand(0.3, -0.4)


def test_unselected_nonzero_source_never_mixes_into_output() -> None:
    approach = select(
        CommandSource.APPROACH,
        approach=PlanarCommand(0.1, 0.2),
        cooperation=PlanarCommand(99.0, -99.0),
    )
    cooperation = select(
        CommandSource.COOPERATION,
        approach=PlanarCommand(99.0, 99.0),
        cooperation=PlanarCommand(0.3, -0.4),
    )
    assert approach == PlanarCommand(0.1, 0.2)
    assert cooperation == PlanarCommand(0.3, -0.4)


def test_selected_approach_stale_is_zero() -> None:
    assert select(
        CommandSource.APPROACH, approach_received=9.649
    ) == PlanarCommand()


def test_selected_cooperation_stale_is_zero() -> None:
    assert select(
        CommandSource.COOPERATION, cooperation_received=9.499
    ) == PlanarCommand()


def test_missing_selected_publisher_cache_is_zero() -> None:
    command = select_command(
        CommandSource.APPROACH,
        10.0,
        parameters(),
        approach_command=None,
        approach_received_seconds=None,
        cooperation_command=PlanarCommand(0.3, 0.0),
        cooperation_received_seconds=10.0,
    )
    assert command == PlanarCommand()


def test_freshness_includes_boundary_and_rejects_clock_rollback() -> None:
    assert command_is_fresh(10.35, 10.0, 0.35)
    assert not command_is_fresh(10.351, 10.0, 0.35)
    assert not command_is_fresh(9.9, 10.0, 0.35)
    assert not command_is_fresh(10.0, None, 0.35)


@pytest.mark.parametrize(
    "values",
    [
        (nan, 0.0, 0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0, 0.0, inf),
        (0.0, 0.1, 0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.1, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0, 0.1, 0.0),
    ],
)
def test_invalid_or_nonplanar_input_is_rejected(values) -> None:
    assert sanitize_command(*values, axis_epsilon=1.0e-9) is None


def test_planar_input_is_preserved_without_clamping() -> None:
    assert sanitize_command(
        0.7, 0.0, 0.0, 0.0, 0.0, -1.2, 1.0e-9
    ) == PlanarCommand(0.7, -1.2)


@pytest.mark.parametrize(
    "invalid",
    [
        parameters(approach_timeout=0.0),
        parameters(cooperation_timeout=-1.0),
        parameters(axis_epsilon=-1.0),
        parameters(approach_timeout=nan),
    ],
)
def test_invalid_parameters_are_rejected(invalid: SelectorParameters) -> None:
    with pytest.raises(ValueError):
        invalid.validate()
