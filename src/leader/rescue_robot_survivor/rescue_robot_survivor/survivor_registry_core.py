"""ROS-independent persistent survivor registry logic."""

from dataclasses import dataclass
from enum import IntEnum
from math import hypot, isfinite
from typing import Iterable, Sequence, Tuple


class TrackStatus(IntEnum):
    """Public states shared with SurvivorTrack.msg."""

    CONFIRMED = 1
    LOST = 2


@dataclass(frozen=True)
class Position3D:
    """A map-frame position in meters."""

    x: float
    y: float
    z: float

    @property
    def is_finite(self) -> bool:
        """Return whether all coordinates can safely enter the registry."""
        return all(isfinite(value) for value in (self.x, self.y, self.z))


@dataclass(frozen=True)
class RegistryConfig:
    """Validated algorithm parameters."""

    association_radius_m: float = 0.50
    reassociation_radius_m: float = 0.75
    confirm_hits: int = 3
    tentative_timeout_sec: float = 2.0
    visible_timeout_sec: float = 2.0
    position_ema_alpha: float = 0.50

    def __post_init__(self) -> None:
        """Reject unsafe configuration instead of silently substituting."""
        for name, value in (
            ("association_radius_m", self.association_radius_m),
            ("reassociation_radius_m", self.reassociation_radius_m),
            ("tentative_timeout_sec", self.tentative_timeout_sec),
            ("visible_timeout_sec", self.visible_timeout_sec),
        ):
            if not isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if (
            not isinstance(self.confirm_hits, int)
            or isinstance(self.confirm_hits, bool)
            or self.confirm_hits < 1
        ):
            raise ValueError("confirm_hits must be a positive integer")
        if (
            not isfinite(self.position_ema_alpha)
            or not 0.0 < self.position_ema_alpha <= 1.0
        ):
            raise ValueError("position_ema_alpha must be in (0.0, 1.0]")


@dataclass(frozen=True)
class TrackSnapshot:
    """Immutable public view of one confirmed or lost survivor."""

    survivor_id: int
    raw_position: Position3D
    filtered_position: Position3D
    status: TrackStatus
    visible: bool
    observation_count: int
    first_seen_ns: int
    last_seen_ns: int


@dataclass(frozen=True)
class UpdateResult:
    """Validation result for one detection array."""

    accepted: bool
    valid_detection_count: int
    invalid_detection_count: int


@dataclass
class _TentativeTrack:
    internal_id: int
    raw_position: Position3D
    filtered_position: Position3D
    first_seen_ns: int
    last_seen_ns: int
    hit_count: int = 1


@dataclass
class _ConfirmedTrack:
    survivor_id: int
    raw_position: Position3D
    filtered_position: Position3D
    first_seen_ns: int
    last_seen_ns: int
    observation_count: int
    visible: bool = True
    status: TrackStatus = TrackStatus.CONFIRMED


class SurvivorRegistry:
    """Associate map detections and retain mission-runtime survivor IDs."""

    def __init__(self, config: RegistryConfig = RegistryConfig()) -> None:
        self._config = config
        self._tentative_tracks = {}
        self._confirmed_tracks = {}
        self._next_tentative_id = 1
        self._next_survivor_id = 1
        self._last_input_stamp_ns = None

    @property
    def tentative_count(self) -> int:
        """Return the number of unpublished candidates."""
        return len(self._tentative_tracks)

    @property
    def next_survivor_id(self) -> int:
        """Expose the next ID for deterministic tests and diagnostics."""
        return self._next_survivor_id

    def snapshots(self) -> Tuple[TrackSnapshot, ...]:
        """Return confirmed and lost tracks sorted by public ID."""
        return tuple(
            TrackSnapshot(
                survivor_id=track.survivor_id,
                raw_position=track.raw_position,
                filtered_position=track.filtered_position,
                status=track.status,
                visible=track.visible,
                observation_count=track.observation_count,
                first_seen_ns=track.first_seen_ns,
                last_seen_ns=track.last_seen_ns,
            )
            for track in sorted(
                self._confirmed_tracks.values(),
                key=lambda item: item.survivor_id,
            )
        )

    def update(
        self,
        detections: Iterable[Position3D],
        timestamp_ns: int,
    ) -> UpdateResult:
        """Validate and associate one timestamped detection array."""
        detection_list = tuple(detections)
        valid_detections = tuple(
            detection for detection in detection_list
            if isinstance(detection, Position3D) and detection.is_finite
        )
        invalid_count = len(detection_list) - len(valid_detections)
        if not self._valid_timestamp(timestamp_ns):
            return UpdateResult(False, 0, invalid_count)
        if (
            self._last_input_stamp_ns is not None
            and timestamp_ns <= self._last_input_stamp_ns
        ):
            return UpdateResult(False, 0, invalid_count)

        self.advance_time(timestamp_ns)
        unmatched = set(range(len(valid_detections)))

        confirmed_matches = self._match_confirmed(
            valid_detections, unmatched
        )
        for track, detection_index in confirmed_matches:
            self._update_confirmed(
                track, valid_detections[detection_index], timestamp_ns
            )
            unmatched.remove(detection_index)

        tentative_matches = self._match_tentative(
            valid_detections, unmatched
        )
        for track, detection_index in tentative_matches:
            self._update_tentative(
                track, valid_detections[detection_index], timestamp_ns
            )
            unmatched.remove(detection_index)

        for detection_index in sorted(
            unmatched,
            key=lambda index: self._detection_key(
                valid_detections[index], index
            ),
        ):
            self._create_tentative(
                valid_detections[detection_index], timestamp_ns
            )

        self._promote_confirmed()
        self._last_input_stamp_ns = timestamp_ns
        return UpdateResult(
            True, len(valid_detections), invalid_count
        )

    def advance_time(self, timestamp_ns: int) -> bool:
        """Expire tentative tracks and mark stale confirmed tracks LOST."""
        if not self._valid_timestamp(timestamp_ns):
            return False
        changed = False
        tentative_timeout_ns = self._seconds_to_ns(
            self._config.tentative_timeout_sec
        )
        for internal_id, track in tuple(self._tentative_tracks.items()):
            if timestamp_ns - track.first_seen_ns > tentative_timeout_ns:
                del self._tentative_tracks[internal_id]
                changed = True

        visible_timeout_ns = self._seconds_to_ns(
            self._config.visible_timeout_sec
        )
        for track in self._confirmed_tracks.values():
            if (
                track.visible
                and timestamp_ns - track.last_seen_ns > visible_timeout_ns
            ):
                track.visible = False
                track.status = TrackStatus.LOST
                changed = True
        return changed

    def reset(self) -> None:
        """Clear one mapping-session registry and restart public IDs."""
        self._tentative_tracks.clear()
        self._confirmed_tracks.clear()
        self._next_tentative_id = 1
        self._next_survivor_id = 1
        self._last_input_stamp_ns = None

    def _match_confirmed(
        self,
        detections: Sequence[Position3D],
        detection_indices: set,
    ):
        candidates = []
        for track in self._confirmed_tracks.values():
            if track.visible:
                reference = track.raw_position
                radius = self._config.association_radius_m
            else:
                reference = track.filtered_position
                radius = self._config.reassociation_radius_m
            for detection_index in detection_indices:
                detection = detections[detection_index]
                distance = self._distance_xy(reference, detection)
                if distance <= radius:
                    candidates.append((
                        distance,
                        track.survivor_id,
                        *self._detection_key(detection, detection_index),
                        track,
                        detection_index,
                    ))
        return self._select_one_to_one(candidates)

    def _match_tentative(
        self,
        detections: Sequence[Position3D],
        detection_indices: set,
    ):
        candidates = []
        for track in self._tentative_tracks.values():
            for detection_index in detection_indices:
                detection = detections[detection_index]
                distance = self._distance_xy(
                    track.raw_position, detection
                )
                if distance <= self._config.association_radius_m:
                    candidates.append((
                        distance,
                        track.internal_id,
                        *self._detection_key(detection, detection_index),
                        track,
                        detection_index,
                    ))
        return self._select_one_to_one(candidates)

    @staticmethod
    def _select_one_to_one(candidates):
        matched_track_ids = set()
        matched_detection_indices = set()
        matches = []
        for candidate in sorted(candidates, key=lambda item: item[:-2]):
            track = candidate[-2]
            detection_index = candidate[-1]
            track_key = candidate[1]
            if (
                track_key in matched_track_ids
                or detection_index in matched_detection_indices
            ):
                continue
            matched_track_ids.add(track_key)
            matched_detection_indices.add(detection_index)
            matches.append((track, detection_index))
        return matches

    def _update_confirmed(
        self,
        track: _ConfirmedTrack,
        detection: Position3D,
        timestamp_ns: int,
    ) -> None:
        track.raw_position = detection
        track.filtered_position = self._ema(
            track.filtered_position, detection
        )
        track.last_seen_ns = timestamp_ns
        track.observation_count += 1
        track.visible = True
        track.status = TrackStatus.CONFIRMED

    def _update_tentative(
        self,
        track: _TentativeTrack,
        detection: Position3D,
        timestamp_ns: int,
    ) -> None:
        track.raw_position = detection
        track.filtered_position = self._ema(
            track.filtered_position, detection
        )
        track.last_seen_ns = timestamp_ns
        track.hit_count += 1

    def _create_tentative(
        self, detection: Position3D, timestamp_ns: int
    ) -> None:
        internal_id = self._next_tentative_id
        self._next_tentative_id += 1
        self._tentative_tracks[internal_id] = _TentativeTrack(
            internal_id=internal_id,
            raw_position=detection,
            filtered_position=detection,
            first_seen_ns=timestamp_ns,
            last_seen_ns=timestamp_ns,
        )

    def _promote_confirmed(self) -> None:
        ready = sorted(
            (
                track for track in self._tentative_tracks.values()
                if track.hit_count >= self._config.confirm_hits
            ),
            key=lambda track: track.internal_id,
        )
        for tentative in ready:
            survivor_id = self._next_survivor_id
            self._next_survivor_id += 1
            self._confirmed_tracks[survivor_id] = _ConfirmedTrack(
                survivor_id=survivor_id,
                raw_position=tentative.raw_position,
                filtered_position=tentative.filtered_position,
                first_seen_ns=tentative.first_seen_ns,
                last_seen_ns=tentative.last_seen_ns,
                observation_count=tentative.hit_count,
            )
            del self._tentative_tracks[tentative.internal_id]

    def _ema(
        self, previous: Position3D, current: Position3D
    ) -> Position3D:
        alpha = self._config.position_ema_alpha
        inverse = 1.0 - alpha
        return Position3D(
            alpha * current.x + inverse * previous.x,
            alpha * current.y + inverse * previous.y,
            alpha * current.z + inverse * previous.z,
        )

    @staticmethod
    def _distance_xy(first: Position3D, second: Position3D) -> float:
        return hypot(second.x - first.x, second.y - first.y)

    @staticmethod
    def _detection_key(
        detection: Position3D, original_index: int
    ) -> tuple:
        return (detection.x, detection.y, detection.z, original_index)

    @staticmethod
    def _seconds_to_ns(seconds: float) -> int:
        return int(seconds * 1_000_000_000)

    @staticmethod
    def _valid_timestamp(timestamp_ns: int) -> bool:
        return (
            isinstance(timestamp_ns, int)
            and not isinstance(timestamp_ns, bool)
            and timestamp_ns > 0
        )
