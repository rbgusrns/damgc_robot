"""Persistent survivor registry algorithm contracts."""

from math import inf, nan

import pytest

from rescue_robot_survivor.survivor_registry_core import (
    Position3D,
    RegistryConfig,
    SurvivorRegistry,
    TrackStatus,
)


SECOND = 1_000_000_000


def position(x, y=0.0, z=0.5):
    """Create one map detection."""
    return Position3D(x, y, z)


def observe(registry, timestamp_sec, *detections):
    """Submit detections using exact integer nanoseconds."""
    return registry.update(detections, int(timestamp_sec * SECOND))


def confirm_one(registry, x=0.0):
    """Confirm one stationary survivor at the default boundaries."""
    observe(registry, 1.0, position(x))
    observe(registry, 1.7, position(x))
    observe(registry, 2.4, position(x))
    observe(registry, 3.0, position(x))
    return registry.snapshots()[0]


def immediate_config(**overrides):
    """Disable confirmation delay for tests of unrelated behavior."""
    values = {
        "confirm_min_duration_sec": 0.0,
        "confirm_min_hits": 1,
    }
    values.update(overrides)
    return RegistryConfig(**values)


def test_repeated_detection_confirms_one_public_id():
    registry = SurvivorRegistry()

    confirmed = confirm_one(registry, 1.0)

    assert confirmed.survivor_id == 1
    assert confirmed.observation_count == 4
    assert confirmed.visible
    assert confirmed.status == TrackStatus.CONFIRMED
    assert registry.tentative_count == 0
    assert registry.next_survivor_id == 2


def test_default_robustness_parameters_preserve_spatial_tuning():
    config = RegistryConfig()

    assert config.confirm_min_duration_sec == 2.0
    assert config.confirm_min_hits == 4
    assert config.tentative_max_gap_sec == 0.8
    assert config.visible_timeout_sec == 4.0
    assert config.association_radius_m == 0.50
    assert config.reassociation_radius_m == 0.75
    assert config.position_ema_alpha == 0.50


def test_four_hits_within_one_second_do_not_confirm():
    registry = SurvivorRegistry()

    for stamp in (1.0, 1.3, 1.6, 1.9):
        observe(registry, stamp, position(0.0))

    assert registry.snapshots() == ()
    assert registry.tentative_count == 1


def test_many_hits_below_minimum_duration_do_not_confirm():
    registry = SurvivorRegistry()

    for stamp in (1.0, 1.3, 1.6, 1.9, 2.2, 2.5):
        observe(registry, stamp, position(0.0))

    assert registry.snapshots() == ()
    assert registry.tentative_count == 1


def test_minimum_duration_without_minimum_hits_does_not_confirm():
    registry = SurvivorRegistry(RegistryConfig(confirm_min_hits=5))

    for stamp in (1.0, 1.7, 2.4, 3.0):
        observe(registry, stamp, position(0.0))

    assert registry.snapshots() == ()
    assert registry.tentative_count == 1


def test_exact_minimum_duration_and_hits_confirm():
    registry = SurvivorRegistry()

    for stamp in (1.0, 1.7, 2.4):
        observe(registry, stamp, position(0.0))
    assert registry.snapshots() == ()
    observe(registry, 3.0, position(0.0))

    assert len(registry.snapshots()) == 1
    assert registry.tentative_count == 0


def test_exact_tentative_max_gap_keeps_track_and_allows_promotion():
    registry = SurvivorRegistry(RegistryConfig(
        confirm_min_duration_sec=0.0,
        confirm_min_hits=2,
    ))

    registry.update((position(0.0),), 1_000_000_000)
    registry.update((position(0.0),), 1_800_000_000)

    assert len(registry.snapshots()) == 1
    assert registry.snapshots()[0].observation_count == 2


def test_gap_over_tentative_max_expires_before_association():
    registry = SurvivorRegistry(RegistryConfig(
        confirm_min_duration_sec=0.0,
        confirm_min_hits=2,
    ))

    registry.update((position(0.0),), 1_000_000_000)
    registry.update((position(0.0),), 1_800_000_001)

    assert registry.snapshots() == ()
    assert registry.tentative_count == 1


def test_tentative_expiry_uses_last_seen_not_first_seen():
    registry = SurvivorRegistry(RegistryConfig(confirm_min_hits=10))
    for stamp in (1.0, 1.7, 2.4):
        observe(registry, stamp, position(0.0))

    registry.advance_time(3_100_000_000)

    assert registry.tentative_count == 1


def test_long_duration_with_broken_continuity_does_not_confirm():
    registry = SurvivorRegistry(RegistryConfig(
        confirm_min_hits=2,
    ))

    observe(registry, 1.0, position(0.0))
    observe(registry, 3.1, position(0.0))

    assert registry.snapshots() == ()
    assert registry.tentative_count == 1


def test_single_detection_never_gets_a_public_id_and_expires():
    registry = SurvivorRegistry()
    observe(registry, 1.0, position(0.0))

    assert registry.snapshots() == ()
    assert registry.tentative_count == 1

    registry.advance_time(1_800_000_001)
    assert registry.tentative_count == 0
    assert registry.next_survivor_id == 1


def test_far_second_person_gets_a_new_id():
    registry = SurvivorRegistry()
    confirm_one(registry, 0.0)

    observe(registry, 4.0, position(2.0))
    observe(registry, 4.7, position(2.0))
    observe(registry, 5.4, position(2.0))
    observe(registry, 6.0, position(2.0))

    tracks = registry.snapshots()
    assert [track.survivor_id for track in tracks] == [1, 2]
    assert tracks[0].raw_position == position(0.0)
    assert tracks[1].raw_position == position(2.0)


def test_two_people_confirm_as_two_ids():
    registry = SurvivorRegistry()
    for stamp in (1.0, 1.7, 2.4, 3.0):
        observe(registry, stamp, position(-1.0), position(1.0))

    tracks = registry.snapshots()
    assert len(tracks) == 2
    assert [track.survivor_id for track in tracks] == [1, 2]
    assert [track.raw_position.x for track in tracks] == [-1.0, 1.0]


def test_reversing_input_order_preserves_map_associated_ids():
    registry = SurvivorRegistry()
    observe(registry, 1.0, position(-1.0), position(1.0))
    observe(registry, 1.7, position(1.0), position(-1.0))
    observe(registry, 2.4, position(-1.0), position(1.0))
    observe(registry, 3.0, position(1.0), position(-1.0))

    before = {
        track.raw_position.x: track.survivor_id
        for track in registry.snapshots()
    }
    observe(registry, 3.2, position(1.05), position(-1.05))
    after = {
        round(track.raw_position.x): track.survivor_id
        for track in registry.snapshots()
    }

    assert before == {-1.0: 1, 1.0: 2}
    assert after == {-1: 1, 1: 2}


def test_one_detection_updates_at_most_one_track():
    registry = SurvivorRegistry(immediate_config(
        association_radius_m=0.50,
        reassociation_radius_m=0.75,
    ))
    observe(registry, 1.0, position(0.0), position(0.8))

    observe(registry, 1.2, position(0.4))

    tracks = registry.snapshots()
    assert [track.observation_count for track in tracks] == [2, 1]


def test_two_detections_cannot_both_update_one_track():
    registry = SurvivorRegistry(immediate_config())
    observe(registry, 1.0, position(0.0))

    observe(registry, 1.2, position(0.1), position(0.2))

    tracks = registry.snapshots()
    assert tracks[0].observation_count == 2
    assert tracks[0].raw_position == position(0.1)
    assert tracks[1].observation_count == 1
    assert tracks[1].raw_position == position(0.2)


def test_confirmed_track_becomes_lost_without_deletion_or_motion():
    registry = SurvivorRegistry()
    confirmed = confirm_one(registry)

    registry.advance_time(7 * SECOND + 1)

    lost = registry.snapshots()[0]
    assert lost.survivor_id == confirmed.survivor_id
    assert not lost.visible
    assert lost.status == TrackStatus.LOST
    assert lost.raw_position == confirmed.raw_position
    assert lost.filtered_position == confirmed.filtered_position


def test_visible_timeout_keeps_track_through_exact_boundary():
    registry = SurvivorRegistry()
    confirm_one(registry)

    registry.advance_time(6_900_000_000)
    before_boundary = registry.snapshots()[0]
    assert before_boundary.visible
    assert before_boundary.status == TrackStatus.CONFIRMED

    registry.advance_time(7_000_000_000)
    at_boundary = registry.snapshots()[0]
    assert at_boundary.visible
    assert at_boundary.status == TrackStatus.CONFIRMED

    registry.advance_time(7_000_000_001)
    after_boundary = registry.snapshots()[0]
    assert not after_boundary.visible
    assert after_boundary.status == TrackStatus.LOST


def test_lost_track_reassociates_to_the_same_id():
    registry = SurvivorRegistry()
    confirm_one(registry)
    registry.advance_time(7 * SECOND + 1)

    observe(registry, 7.1, position(0.6))

    track = registry.snapshots()[0]
    assert track.survivor_id == 1
    assert track.visible
    assert track.status == TrackStatus.CONFIRMED
    assert track.observation_count == 5
    assert registry.tentative_count == 0


def test_far_reappearance_starts_new_candidate_then_new_id():
    registry = SurvivorRegistry()
    confirm_one(registry)
    registry.advance_time(7 * SECOND + 1)

    observe(registry, 7.1, position(1.0))
    assert len(registry.snapshots()) == 1
    assert registry.tentative_count == 1
    observe(registry, 7.8, position(1.0))
    observe(registry, 8.5, position(1.0))
    observe(registry, 9.1, position(1.0))

    tracks = registry.snapshots()
    assert [track.survivor_id for track in tracks] == [1, 2]
    assert tracks[0].status == TrackStatus.LOST


def test_visible_movement_keeps_id_and_updates_position():
    registry = SurvivorRegistry()
    confirm_one(registry)

    observe(registry, 3.2, position(0.4))
    observe(registry, 3.4, position(0.8))

    track = registry.snapshots()[0]
    assert track.survivor_id == 1
    assert track.raw_position.x == pytest.approx(0.8)
    assert track.filtered_position.x == pytest.approx(0.5)
    assert track.observation_count == 6


def test_lost_position_remains_frozen_across_timer_ticks():
    registry = SurvivorRegistry()
    confirm_one(registry, 0.25)
    before = registry.snapshots()[0]

    registry.advance_time(7 * SECOND + 1)
    registry.advance_time(10 * SECOND)

    after = registry.snapshots()[0]
    assert after.raw_position == before.raw_position
    assert after.filtered_position == before.filtered_position


def test_ema_filters_all_coordinates_and_alpha_one_disables_filter():
    registry = SurvivorRegistry(immediate_config(
        position_ema_alpha=0.25
    ))
    observe(registry, 1.0, Position3D(0.0, 0.0, 0.0))
    observe(registry, 1.2, Position3D(0.4, 0.2, 0.8))
    filtered = registry.snapshots()[0].filtered_position
    assert filtered == Position3D(0.1, 0.05, 0.2)

    unfiltered = SurvivorRegistry(immediate_config(
        position_ema_alpha=1.0
    ))
    observe(unfiltered, 1.0, position(0.0))
    observe(unfiltered, 1.2, position(0.4))
    assert unfiltered.snapshots()[0].filtered_position == position(0.4)


def test_non_finite_detections_are_skipped_without_hiding_valid_ones():
    registry = SurvivorRegistry(immediate_config())

    result = observe(
        registry,
        1.0,
        position(0.0),
        position(nan),
        position(inf),
        Position3D(0.0, -inf, 0.5),
    )

    assert result.accepted
    assert result.valid_detection_count == 1
    assert result.invalid_detection_count == 3
    assert len(registry.snapshots()) == 1


def test_empty_and_invalid_timestamp_inputs_are_safe():
    registry = SurvivorRegistry()

    empty = observe(registry, 1.0)
    invalid = registry.update((position(0.0),), 0)
    duplicate = observe(registry, 1.0, position(0.0))

    assert empty.accepted
    assert not invalid.accepted
    assert not duplicate.accepted
    assert registry.snapshots() == ()
    assert registry.tentative_count == 0


def test_reset_clears_all_state_and_restarts_public_id_at_one():
    registry = SurvivorRegistry(immediate_config())
    observe(registry, 1.0, position(0.0), position(2.0))
    observe(registry, 1.2, position(4.0))
    assert registry.next_survivor_id == 4

    registry.reset()

    assert registry.snapshots() == ()
    assert registry.tentative_count == 0
    assert registry.next_survivor_id == 1
    observe(registry, 1.0, position(9.0))
    assert registry.snapshots()[0].survivor_id == 1


@pytest.mark.parametrize(
    "overrides",
    [
        {"association_radius_m": 0.0},
        {"reassociation_radius_m": inf},
        {"confirm_min_duration_sec": -0.1},
        {"confirm_min_duration_sec": nan},
        {"confirm_min_duration_sec": inf},
        {"confirm_min_hits": 0},
        {"confirm_min_hits": 1.5},
        {"confirm_min_hits": True},
        {"tentative_max_gap_sec": 0.0},
        {"tentative_max_gap_sec": inf},
        {"visible_timeout_sec": 0.0},
        {"visible_timeout_sec": nan},
        {"position_ema_alpha": 0.0},
        {"position_ema_alpha": 1.1},
        {"position_ema_alpha": nan},
    ],
)
def test_invalid_configuration_is_rejected(overrides):
    with pytest.raises(ValueError):
        RegistryConfig(**overrides)
