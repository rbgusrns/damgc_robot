"""Camera-free tests for EyeTrack association and temporal eye voting."""

import unittest

from eye_track import (
    EyeStateInput,
    EyeStateSample,
    EyeTrackManager,
    bbox_iou,
    normalized_center_distance,
)


OPEN = EyeStateInput(True, "OPEN", 0.9)
CLOSED = EyeStateInput(True, "CLOSED", 0.9)
INVALID = EyeStateInput(False, "INVALID", 0.0)


def feed(manager, stamp_ns, left, right=None, track_bbox=(100, 100, 100, 100)):
    right = left if right is None else right
    return manager.update(stamp_ns, [track_bbox], [(left, right)])[0]


class EyeTrackManagerTests(unittest.TestCase):
    def test_two_faces_keep_ids_when_moving_and_detection_order_changes(self):
        manager = EyeTrackManager()
        first = manager.update(1_000_000_000,
                               [(100, 100, 100, 100), (400, 100, 100, 100)])
        self.assertEqual(first, (0, 1))
        manager.tracks[0].left_history.append(
            EyeStateSample(1_000_000_000, True, "OPEN", 0.9))
        manager.tracks[1].left_history.append(
            EyeStateSample(1_000_000_000, True, "CLOSED", 0.9))
        second = manager.update(1_500_000_000,
                                [(385, 105, 100, 100), (115, 105, 100, 100)])
        self.assertEqual(second, (1, 0))
        self.assertEqual(manager.tracks[0].left_history[0].state, "OPEN")
        self.assertEqual(manager.tracks[1].left_history[0].state, "CLOSED")

    def test_missing_face_remains_until_image_timestamp_timeout(self):
        manager = EyeTrackManager(track_timeout_sec=1.5)
        self.assertEqual(manager.update(0, [(100, 100, 100, 100)]), (0,))
        self.assertEqual(manager.update(1_000_000_000, []), ())
        self.assertIn(0, manager.tracks)
        self.assertEqual(manager.update(1_500_000_000,
                                        [(110, 100, 100, 100)]), (0,))
        self.assertEqual(manager.update(3_000_000_001, []), ())
        self.assertNotIn(0, manager.tracks)
        self.assertEqual(manager.update(3_100_000_000,
                                        [(110, 100, 100, 100)]), (1,))

    def test_one_old_track_cannot_match_two_faces_in_one_frame(self):
        manager = EyeTrackManager()
        self.assertEqual(manager.update(0, [(100, 100, 100, 100)]), (0,))
        assignments = manager.update(500_000_000,
                                     [(105, 100, 100, 100),
                                      (110, 100, 100, 100)])
        self.assertEqual(assignments, (0, 1))
        self.assertEqual(len(set(assignments)), 2)

    def test_nonoverlapping_narrow_boxes_can_match_by_center_distance(self):
        manager = EyeTrackManager()
        first = (100, 100, 10, 100)
        second = (110, 100, 10, 100)
        self.assertEqual(bbox_iou(first, second), 0.0)
        self.assertLess(normalized_center_distance(first, second), 0.30)
        self.assertEqual(manager.update(0, [first]), (0,))
        self.assertEqual(manager.update(500_000_000, [second]), (0,))

    def test_large_size_change_creates_new_track(self):
        manager = EyeTrackManager()
        self.assertEqual(manager.update(0, [(100, 100, 100, 100)]), (0,))
        self.assertEqual(manager.update(500_000_000,
                                        [(0, 0, 300, 300)]), (1,))
        self.assertIn(0, manager.tracks)

    def test_invalid_bbox_does_not_create_track(self):
        manager = EyeTrackManager()
        self.assertEqual(manager.update(0, [None, (0, 0, 0, 100),
                                            (0, 0, float("nan"), 1),
                                            (100, 100, 100, 100)]),
                         (None, None, None, 0))
        self.assertEqual(list(manager.tracks), [0])

    def test_timestamps_must_advance_and_reset_does_not_reuse_ids(self):
        manager = EyeTrackManager()
        manager.update(1_000_000_000, [(0, 0, 50, 50)])
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            manager.update(1_000_000_000, [])
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            manager.update(999_999_999, [])
        self.assertEqual(manager.tracks[0].last_seen_stamp_ns, 1_000_000_000)
        manager.reset()
        self.assertEqual(manager.update(0, [(0, 0, 50, 50)]), (1,))

    def test_invalid_thresholds_fail_before_tracking(self):
        with self.assertRaisesRegex(ValueError, "track_timeout_sec"):
            EyeTrackManager(track_timeout_sec=0)
        with self.assertRaisesRegex(ValueError, "match_iou_threshold"):
            EyeTrackManager(match_iou_threshold=1.1)
        with self.assertRaisesRegex(ValueError, "match_center_distance_threshold"):
            EyeTrackManager(match_center_distance_threshold=-0.1)

    def test_blink_vote_keeps_open_and_closed_excursion_does_not_dominate(self):
        manager = EyeTrackManager()
        for index, state in enumerate(
                [OPEN, OPEN, OPEN, CLOSED, OPEN, OPEN]):
            feed(manager, index * 200_000_000, state)
        track = manager.tracks[0]
        self.assertEqual(track.left_stable.state, "OPEN")
        self.assertEqual(track.right_stable.state, "OPEN")
        self.assertEqual(track.combined_state, "OPEN")

        closed_manager = EyeTrackManager()
        for index, state in enumerate(
                [CLOSED, CLOSED, CLOSED, OPEN, CLOSED, CLOSED]):
            feed(closed_manager, index * 200_000_000, state)
        self.assertEqual(closed_manager.tracks[0].combined_state, "CLOSED")

    def test_sustained_closed_transitions_and_reopening_recovers(self):
        manager = EyeTrackManager()
        stamp = 0
        for _ in range(5):
            feed(manager, stamp, OPEN)
            stamp += 200_000_000
        self.assertEqual(manager.tracks[0].combined_state, "OPEN")

        closed_transition_seen = False
        for _ in range(16):
            feed(manager, stamp, CLOSED)
            stamp += 200_000_000
            closed_transition_seen |= manager.tracks[0].combined_state == "CLOSED"
        self.assertTrue(closed_transition_seen)
        self.assertEqual(manager.tracks[0].combined_state, "CLOSED")

        open_transition_seen = False
        for _ in range(16):
            feed(manager, stamp, OPEN)
            stamp += 200_000_000
            open_transition_seen |= manager.tracks[0].combined_state == "OPEN"
        self.assertTrue(open_transition_seen)
        self.assertEqual(manager.tracks[0].combined_state, "OPEN")

    def test_ambiguous_vote_is_unknown(self):
        manager = EyeTrackManager()
        for index, state in enumerate([OPEN, CLOSED, OPEN, CLOSED]):
            feed(manager, index * 200_000_000, state)
        self.assertEqual(manager.tracks[0].left_stable.state, "UNKNOWN")

    def test_too_few_valid_samples_stays_unknown(self):
        manager = EyeTrackManager(min_valid_samples=5)
        for index in range(4):
            feed(manager, index * 200_000_000, OPEN)
        self.assertEqual(manager.tracks[0].left_stable.valid_sample_count, 4)
        self.assertEqual(manager.tracks[0].combined_state, "UNKNOWN")

    def test_invalid_samples_do_not_vote_and_low_coverage_is_unknown(self):
        manager = EyeTrackManager(min_valid_samples=5, min_valid_coverage=0.6)
        for index in range(11):
            feed(manager, index * 200_000_000,
                 OPEN if index < 5 else INVALID)
        result = manager.tracks[0].left_stable
        self.assertEqual(result.valid_sample_count, 5)
        self.assertLess(result.valid_coverage, 0.6)
        self.assertEqual(result.state, "UNKNOWN")

    def test_combined_requires_both_eyes_to_agree(self):
        manager = EyeTrackManager()
        for index in range(5):
            feed(manager, index * 200_000_000, OPEN,
                 INVALID if index < 4 else OPEN)
        self.assertEqual(manager.tracks[0].left_stable.state, "OPEN")
        self.assertEqual(manager.tracks[0].right_stable.state, "UNKNOWN")
        self.assertEqual(manager.tracks[0].combined_state, "UNKNOWN")

        for index in range(5, 10):
            feed(manager, index * 200_000_000, OPEN, CLOSED)
        self.assertEqual(manager.tracks[0].combined_state, "UNKNOWN")

    def test_two_tracks_keep_independent_temporal_histories(self):
        manager = EyeTrackManager()
        boxes = [(100, 100, 100, 100), (400, 100, 100, 100)]
        for index in range(6):
            states = [(OPEN, OPEN), (CLOSED, CLOSED)]
            assignments = manager.update(index * 200_000_000, boxes, states)
        self.assertEqual(assignments, (0, 1))
        self.assertEqual(manager.tracks[0].combined_state, "OPEN")
        self.assertEqual(manager.tracks[1].combined_state, "CLOSED")

    def test_history_uses_time_window_and_evicts_old_samples(self):
        manager = EyeTrackManager(history_window_sec=3.0)
        for index in range(5):
            feed(manager, index * 500_000_000, OPEN)
        feed(manager, 3_000_000_001, CLOSED)
        history = manager.tracks[0].left_history
        self.assertEqual(history[0].timestamp_ns, 500_000_000)
        self.assertEqual(len(history), 5)

    def test_weighted_vote_uses_confidence(self):
        manager = EyeTrackManager()
        weak_open = EyeStateInput(True, "OPEN", 0.2)
        strong_closed = EyeStateInput(True, "CLOSED", 0.95)
        for index, state in enumerate([weak_open, weak_open, strong_closed,
                                       strong_closed, strong_closed]):
            feed(manager, index * 200_000_000, state)
        self.assertEqual(manager.tracks[0].left_stable.state, "CLOSED")


if __name__ == "__main__":
    unittest.main()
