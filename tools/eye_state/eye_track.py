"""Short-lived, deterministic face association for Eye State observations.

EyeTrack IDs are temporary process-local IDs. They are NOT Survivor IDs.
This module matches faces and computes short-window per-eye temporal states.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Sequence, Tuple


BBox = Tuple[float, float, float, float]


@dataclass
class EyeStateInput:
    """One raw per-eye classifier result supplied for a frame."""

    valid: bool
    state: str
    confidence: float


@dataclass(frozen=True)
class EyeStateSample:
    """One timestamped raw sample retained inside the temporal window."""

    timestamp_ns: int
    valid: bool
    state: str
    confidence: float


@dataclass(frozen=True)
class StableEyeState:
    """Current temporal result and compact diagnostics for one eye."""

    state: str
    valid_sample_count: int
    history_duration_sec: float
    open_ratio: float
    closed_ratio: float
    valid_coverage: float


def _stable_eye_state(history: Sequence[EyeStateSample], *,
                      min_valid_samples: int,
                      min_valid_coverage: float,
                      open_ratio_threshold: float,
                      closed_ratio_threshold: float) -> StableEyeState:
    classified = [sample for sample in history
                  if sample.valid and sample.state in ("OPEN", "CLOSED")]
    open_score = sum(sample.confidence for sample in classified
                     if sample.state == "OPEN")
    closed_score = sum(sample.confidence for sample in classified
                       if sample.state == "CLOSED")
    total_score = open_score + closed_score
    open_ratio = open_score / total_score if total_score > 0.0 else 0.0
    closed_ratio = closed_score / total_score if total_score > 0.0 else 0.0
    coverage = len(classified) / len(history) if history else 0.0
    duration = ((history[-1].timestamp_ns - history[0].timestamp_ns) / 1e9
                if len(history) > 1 else 0.0)

    state = "UNKNOWN"
    if (len(classified) >= min_valid_samples
            and coverage >= min_valid_coverage
            and total_score > 0.0):
        if open_ratio >= open_ratio_threshold:
            state = "OPEN"
        elif closed_ratio >= closed_ratio_threshold:
            state = "CLOSED"
    return StableEyeState(state, len(classified), duration,
                          open_ratio, closed_ratio, coverage)


@dataclass
class EyeTrack:
    """One temporarily associated face with reserved per-eye histories."""

    track_id: int
    last_bbox: BBox
    last_seen_stamp_ns: int
    left_history: List[EyeStateSample] = field(default_factory=list)
    right_history: List[EyeStateSample] = field(default_factory=list)
    left_stable: StableEyeState = field(default_factory=lambda: StableEyeState(
        "UNKNOWN", 0, 0.0, 0.0, 0.0, 0.0))
    right_stable: StableEyeState = field(default_factory=lambda: StableEyeState(
        "UNKNOWN", 0, 0.0, 0.0, 0.0, 0.0))

    @property
    def combined_state(self) -> str:
        if self.left_stable.state == self.right_stable.state:
            if self.left_stable.state in ("OPEN", "CLOSED"):
                return self.left_stable.state
        return "UNKNOWN"


def _valid_bbox(values: Sequence[float]) -> Optional[BBox]:
    try:
        size = len(values)
    except TypeError:
        return None
    if size != 4:
        return None
    try:
        x, y, width, height = (float(value) for value in values)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (x, y, width, height)):
        return None
    if width <= 0.0 or height <= 0.0:
        return None
    return x, y, width, height


def bbox_iou(first: BBox, second: BBox) -> float:
    """Intersection over union for (x, y, width, height) face boxes."""
    ax, ay, aw, ah = first
    bx, by, bw, bh = second
    overlap_width = max(0.0, min(ax + aw, bx + bw) - max(ax, bx))
    overlap_height = max(0.0, min(ay + ah, by + bh) - max(ay, by))
    intersection = overlap_width * overlap_height
    return intersection / (aw * ah + bw * bh - intersection)


def normalized_center_distance(first: BBox, second: BBox) -> float:
    """Center distance divided by the mean of both bbox diagonals."""
    ax, ay, aw, ah = first
    bx, by, bw, bh = second
    distance = math.hypot((ax + aw / 2) - (bx + bw / 2),
                          (ay + ah / 2) - (by + bh / 2))
    mean_diagonal = (math.hypot(aw, ah) + math.hypot(bw, bh)) / 2
    return distance / mean_diagonal


class EyeTrackManager:
    """Associate faces and calculate independent time-window eye states."""

    def __init__(self, *, track_timeout_sec: float = 1.5,
                 match_iou_threshold: float = 0.25,
                 match_center_distance_threshold: float = 0.30,
                 history_window_sec: float = 3.0,
                 min_valid_samples: int = 5,
                 min_valid_coverage: float = 0.5,
                 open_ratio_threshold: float = 0.70,
                 closed_ratio_threshold: float = 0.70) -> None:
        if not math.isfinite(track_timeout_sec) or track_timeout_sec <= 0:
            raise ValueError("track_timeout_sec must be positive and finite")
        if not math.isfinite(match_iou_threshold) or not 0 <= match_iou_threshold <= 1:
            raise ValueError("match_iou_threshold must be in [0, 1]")
        if (not math.isfinite(match_center_distance_threshold)
                or match_center_distance_threshold < 0):
            raise ValueError("match_center_distance_threshold must be nonnegative")
        if not math.isfinite(history_window_sec) or history_window_sec <= 0:
            raise ValueError("history_window_sec must be positive and finite")
        if isinstance(min_valid_samples, bool) or not isinstance(min_valid_samples, int) or min_valid_samples < 1:
            raise ValueError("min_valid_samples must be a positive integer")
        if (not math.isfinite(min_valid_coverage)
                or not 0.0 <= min_valid_coverage <= 1.0):
            raise ValueError("min_valid_coverage must be in [0, 1]")
        for name, threshold in (("open_ratio_threshold", open_ratio_threshold),
                                ("closed_ratio_threshold", closed_ratio_threshold)):
            if not math.isfinite(threshold) or not 0.5 < threshold <= 1.0:
                raise ValueError(f"{name} must be in (0.5, 1]")
        self.track_timeout_ns = round(track_timeout_sec * 1_000_000_000)
        self.match_iou_threshold = match_iou_threshold
        self.match_center_distance_threshold = match_center_distance_threshold
        self.history_window_ns = round(history_window_sec * 1_000_000_000)
        self.min_valid_samples = min_valid_samples
        self.min_valid_coverage = min_valid_coverage
        self.open_ratio_threshold = open_ratio_threshold
        self.closed_ratio_threshold = closed_ratio_threshold
        self.tracks: Dict[int, EyeTrack] = {}
        self._next_track_id = 0
        self._last_frame_stamp_ns: Optional[int] = None

    def reset(self) -> None:
        """Discard associations after a timestamp reset without reusing IDs."""
        self.tracks.clear()
        self._last_frame_stamp_ns = None

    def update(self, stamp_ns: int, bboxes: Sequence[Sequence[float]],
               eye_states: Optional[Sequence[Tuple[EyeStateInput, EyeStateInput]]] = None
               ) -> Tuple[Optional[int], ...]:
        """Return one EyeTrack ID per input bbox; malformed boxes map to None.

        The timestamp must come from the source image's ROS header. No wall
        clock is used for track expiration or ordering.
        """
        if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) or stamp_ns < 0:
            raise ValueError("stamp_ns must be a nonnegative integer")
        if (self._last_frame_stamp_ns is not None
                and stamp_ns <= self._last_frame_stamp_ns):
            raise ValueError("frame timestamps must be strictly increasing")
        if eye_states is not None and len(eye_states) != len(bboxes):
            raise ValueError("eye_states must contain one left/right pair per bbox")
        self._last_frame_stamp_ns = stamp_ns
        self.tracks = {
            track_id: track for track_id, track in self.tracks.items()
            if stamp_ns - track.last_seen_stamp_ns <= self.track_timeout_ns
        }

        valid_boxes = [_valid_bbox(box) for box in bboxes]
        assignments: List[Optional[int]] = [None] * len(valid_boxes)
        candidates = []
        for face_index, bbox in enumerate(valid_boxes):
            if bbox is None:
                continue
            area = bbox[2] * bbox[3]
            for track_id, track in self.tracks.items():
                old_area = track.last_bbox[2] * track.last_bbox[3]
                area_ratio = area / old_area
                if not 0.5 <= area_ratio <= 2.0:
                    continue
                overlap = bbox_iou(bbox, track.last_bbox)
                center_distance = normalized_center_distance(
                    bbox, track.last_bbox
                )
                if (overlap >= self.match_iou_threshold
                        or center_distance <= self.match_center_distance_threshold):
                    candidates.append((
                        -overlap, center_distance, track_id, face_index
                    ))

        matched_tracks = set()
        for _, _, track_id, face_index in sorted(candidates):
            if assignments[face_index] is not None or track_id in matched_tracks:
                continue
            assignments[face_index] = track_id
            matched_tracks.add(track_id)
            track = self.tracks[track_id]
            track.last_bbox = valid_boxes[face_index]
            track.last_seen_stamp_ns = stamp_ns

        for face_index, bbox in enumerate(valid_boxes):
            if bbox is None or assignments[face_index] is not None:
                continue
            track_id = self._next_track_id
            self._next_track_id += 1
            self.tracks[track_id] = EyeTrack(track_id, bbox, stamp_ns)
            assignments[face_index] = track_id

        for face_index, track_id in enumerate(assignments):
            if track_id is None or eye_states is None:
                continue
            left_input, right_input = eye_states[face_index]
            track = self.tracks[track_id]
            track.left_history.append(self._make_sample(stamp_ns, left_input))
            track.right_history.append(self._make_sample(stamp_ns, right_input))

        for track in self.tracks.values():
            self._trim_and_calculate(track, stamp_ns)
        return tuple(assignments)

    @staticmethod
    def _make_sample(stamp_ns: int, value: EyeStateInput) -> EyeStateSample:
        if not isinstance(value.valid, bool):
            raise ValueError("eye sample valid must be bool")
        if value.state not in ("INVALID", "OPEN", "CLOSED", "LOW_CONFIDENCE",
                               "NO_PREDICTION", "UNKNOWN"):
            raise ValueError(f"unsupported eye state: {value.state!r}")
        confidence = float(value.confidence)
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError("eye confidence must be finite and in [0, 1]")
        return EyeStateSample(stamp_ns, value.valid, value.state, confidence)

    def _trim_and_calculate(self, track: EyeTrack, stamp_ns: int) -> None:
        cutoff_ns = stamp_ns - self.history_window_ns
        track.left_history[:] = [sample for sample in track.left_history
                                 if sample.timestamp_ns >= cutoff_ns]
        track.right_history[:] = [sample for sample in track.right_history
                                  if sample.timestamp_ns >= cutoff_ns]
        options = dict(
            min_valid_samples=self.min_valid_samples,
            min_valid_coverage=self.min_valid_coverage,
            open_ratio_threshold=self.open_ratio_threshold,
            closed_ratio_threshold=self.closed_ratio_threshold,
        )
        track.left_stable = _stable_eye_state(track.left_history, **options)
        track.right_stable = _stable_eye_state(track.right_history, **options)
