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
    """Confirm one stationary survivor with the default three hits."""
    observe(registry, 1.0, position(x))
    observe(registry, 1.2, position(x))
    observe(registry, 1.4, position(x))
    return registry.snapshots()[0]


def test_repeated_detection_confirms_one_public_id():
    registry = SurvivorRegistry()

    confirmed = confirm_one(registry, 1.0)

    assert confirmed.survivor_id == 1
    assert confirmed.observation_count == 3
    assert confirmed.visible
    assert confirmed.status == TrackStatus.CONFIRMED
    assert registry.tentative_count == 0
    assert registry.next_survivor_id == 2


def test_single_detection_never_gets_a_public_id_and_expires():
    registry = SurvivorRegistry()
    observe(registry, 1.0, position(0.0))

    assert registry.snapshots() == ()
    assert registry.tentative_count == 1

    registry.advance_time(3 * SECOND + 1)
    assert registry.tentative_count == 0
    assert registry.next_survivor_id == 1


def test_far_second_person_gets_a_new_id():
    registry = SurvivorRegistry()
    confirm_one(registry, 0.0)

    observe(registry, 2.0, position(2.0))
    observe(registry, 2.2, position(2.0))
    observe(registry, 2.4, position(2.0))

    tracks = registry.snapshots()
    assert [track.survivor_id for track in tracks] == [1, 2]
    assert tracks[0].raw_position == position(0.0)
    assert tracks[1].raw_position == position(2.0)


def test_two_people_confirm_as_two_ids():
    registry = SurvivorRegistry()
    for stamp in (1.0, 1.2, 1.4):
        observe(registry, stamp, position(-1.0), position(1.0))

    tracks = registry.snapshots()
    assert len(tracks) == 2
    assert [track.survivor_id for track in tracks] == [1, 2]
    assert [track.raw_position.x for track in tracks] == [-1.0, 1.0]


def test_reversing_input_order_preserves_map_associated_ids():
    registry = SurvivorRegistry()
    observe(registry, 1.0, position(-1.0), position(1.0))
    observe(registry, 1.2, position(1.0), position(-1.0))
    observe(registry, 1.4, position(-1.0), position(1.0))

    before = {
        track.raw_position.x: track.survivor_id
        for track in registry.snapshots()
    }
    observe(registry, 1.6, position(1.05), position(-1.05))
    after = {
        round(track.raw_position.x): track.survivor_id
        for track in registry.snapshots()
    }

    assert before == {-1.0: 1, 1.0: 2}
    assert after == {-1: 1, 1: 2}


def test_one_detection_updates_at_most_one_track():
    registry = SurvivorRegistry(RegistryConfig(
        association_radius_m=0.50,
        reassociation_radius_m=0.75,
        confirm_hits=1,
    ))
    observe(registry, 1.0, position(0.0), position(0.8))

    observe(registry, 1.2, position(0.4))

    tracks = registry.snapshots()
    assert [track.observation_count for track in tracks] == [2, 1]


def test_two_detections_cannot_both_update_one_track():
    registry = SurvivorRegistry(RegistryConfig(confirm_hits=1))
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

    registry.advance_time(3 * SECOND + 400_000_001)

    lost = registry.snapshots()[0]
    assert lost.survivor_id == confirmed.survivor_id
    assert not lost.visible
    assert lost.status == TrackStatus.LOST
    assert lost.raw_position == confirmed.raw_position
    assert lost.filtered_position == confirmed.filtered_position


def test_lost_track_reassociates_to_the_same_id():
    registry = SurvivorRegistry()
    confirm_one(registry)
    registry.advance_time(4 * SECOND)

    observe(registry, 4.1, position(0.6))

    track = registry.snapshots()[0]
    assert track.survivor_id == 1
    assert track.visible
    assert track.status == TrackStatus.CONFIRMED
    assert track.observation_count == 4
    assert registry.tentative_count == 0


def test_far_reappearance_starts_new_candidate_then_new_id():
    registry = SurvivorRegistry()
    confirm_one(registry)
    registry.advance_time(4 * SECOND)

    observe(registry, 4.1, position(1.0))
    assert len(registry.snapshots()) == 1
    assert registry.tentative_count == 1
    observe(registry, 4.3, position(1.0))
    observe(registry, 4.5, position(1.0))

    tracks = registry.snapshots()
    assert [track.survivor_id for track in tracks] == [1, 2]
    assert tracks[0].status == TrackStatus.LOST


def test_visible_movement_keeps_id_and_updates_position():
    registry = SurvivorRegistry()
    confirm_one(registry)

    observe(registry, 1.6, position(0.4))
    observe(registry, 1.8, position(0.8))

    track = registry.snapshots()[0]
    assert track.survivor_id == 1
    assert track.raw_position.x == pytest.approx(0.8)
    assert track.filtered_position.x == pytest.approx(0.5)
    assert track.observation_count == 5


def test_lost_position_remains_frozen_across_timer_ticks():
    registry = SurvivorRegistry()
    confirm_one(registry, 0.25)
    before = registry.snapshots()[0]

    registry.advance_time(4 * SECOND)
    registry.advance_time(10 * SECOND)

    after = registry.snapshots()[0]
    assert after.raw_position == before.raw_position
    assert after.filtered_position == before.filtered_position


def test_ema_filters_all_coordinates_and_alpha_one_disables_filter():
    registry = SurvivorRegistry(RegistryConfig(
        confirm_hits=1, position_ema_alpha=0.25
    ))
    observe(registry, 1.0, Position3D(0.0, 0.0, 0.0))
    observe(registry, 1.2, Position3D(0.4, 0.2, 0.8))
    filtered = registry.snapshots()[0].filtered_position
    assert filtered == Position3D(0.1, 0.05, 0.2)

    unfiltered = SurvivorRegistry(RegistryConfig(
        confirm_hits=1, position_ema_alpha=1.0
    ))
    observe(unfiltered, 1.0, position(0.0))
    observe(unfiltered, 1.2, position(0.4))
    assert unfiltered.snapshots()[0].filtered_position == position(0.4)


def test_non_finite_detections_are_skipped_without_hiding_valid_ones():
    registry = SurvivorRegistry(RegistryConfig(confirm_hits=1))

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
    registry = SurvivorRegistry(RegistryConfig(confirm_hits=1))
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
        {"confirm_hits": 0},
        {"confirm_hits": 1.5},
        {"tentative_timeout_sec": -1.0},
        {"visible_timeout_sec": nan},
        {"position_ema_alpha": 0.0},
        {"position_ema_alpha": 1.1},
    ],
)
def test_invalid_configuration_is_rejected(overrides):
    with pytest.raises(ValueError):
        RegistryConfig(**overrides)
