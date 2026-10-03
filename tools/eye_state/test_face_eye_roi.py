"""Geometry and quality tests for the landmark-based eye ROI extractor."""

import unittest

import numpy as np

from face_eye_roi import ExtractorConfig, faces_to_eye_rois


def detection(x=10, y=10, width=80, height=80, right=(32, 40),
              left=(64, 40), confidence=0.95):
    """Build one row in YuNet's documented 15-column result layout."""
    return np.asarray([[
        x, y, width, height,
        right[0], right[1], left[0], left[1],
        50, 55, 40, 75, 60, 75, confidence,
    ]], dtype=np.float32)


class FaceEyeRoiTests(unittest.TestCase):
    def setUp(self):
        self.frame = np.zeros((100, 120, 3), dtype=np.uint8)

    def test_no_detection_returns_empty_list(self):
        self.assertEqual(faces_to_eye_rois(self.frame, None), [])
        self.assertEqual(faces_to_eye_rois(self.frame, np.empty((0, 15))), [])

    def test_landmark_order_means_subject_left_is_image_right(self):
        face = faces_to_eye_rois(self.frame, detection())[0]
        self.assertTrue(face.left_eye.valid)
        self.assertTrue(face.right_eye.valid)
        self.assertGreater(face.left_eye.center[0], face.right_eye.center[0])
        self.assertEqual(face.left_eye.side, "left")
        self.assertEqual(face.right_eye.side, "right")

    def test_roi_is_square_and_scaled_by_inter_eye_distance(self):
        face = faces_to_eye_rois(self.frame, detection())[0]
        expected_side = round(32 * 0.85)
        self.assertEqual(face.left_eye.image.shape[:2],
                         (expected_side, expected_side))
        self.assertEqual(face.right_eye.image.shape[:2],
                         (expected_side, expected_side))

    def test_multiple_faces_are_kept_as_separate_frame_results(self):
        rows = np.vstack([detection(), detection(x=30, right=(52, 40),
                                                 left=(84, 40))])
        faces = faces_to_eye_rois(self.frame, rows)
        self.assertEqual(len(faces), 2)
        self.assertTrue(all(face.left_eye.valid and face.right_eye.valid
                            for face in faces))

    def test_low_face_confidence_invalidates_both_eyes(self):
        face = faces_to_eye_rois(self.frame,
                                 detection(confidence=0.59))[0]
        self.assertEqual(face.left_eye.reason,
                         "face_confidence_below_threshold")
        self.assertEqual(face.right_eye.reason,
                         "face_confidence_below_threshold")

    def test_out_of_frame_eye_landmark_is_invalid(self):
        face = faces_to_eye_rois(self.frame,
                                 detection(right=(-1, 40)))[0]
        self.assertEqual(face.right_eye.reason, "invalid_eye_landmark")
        self.assertFalse(face.left_eye.valid)

    def test_too_small_roi_is_invalid(self):
        row = detection(right=(48, 40), left=(52, 40))
        face = faces_to_eye_rois(self.frame, row)[0]
        self.assertEqual(face.left_eye.reason, "roi_below_minimum_size")

    def test_severely_clipped_roi_is_invalid(self):
        row = detection(right=(2, 40), left=(42, 40))
        face = faces_to_eye_rois(self.frame, row)[0]
        self.assertEqual(face.right_eye.reason, "severe_frame_clipping")

    def test_mild_clipping_is_padded_back_to_square(self):
        row = detection(right=(10, 40), left=(50, 40))
        config = ExtractorConfig(min_eye_width=24, min_eye_height=24)
        face = faces_to_eye_rois(self.frame, row, config)[0]
        self.assertTrue(face.right_eye.valid)
        self.assertEqual(face.right_eye.image.shape[:2], (34, 34))
        self.assertGreater(face.right_eye.clipped_fraction, 0.0)

    def test_non_finite_eye_landmark_is_invalid(self):
        row = detection(right=(float("nan"), 40))
        face = faces_to_eye_rois(self.frame, row)[0]
        self.assertEqual(face.right_eye.reason, "invalid_eye_landmark")

    def test_bad_detection_shape_fails_loudly(self):
        with self.assertRaisesRegex(ValueError, "N x 15"):
            faces_to_eye_rois(self.frame, np.zeros((1, 14)))

    def test_invalid_config_fails_loudly(self):
        with self.assertRaisesRegex(ValueError, "roi_side_inter_eye_ratio"):
            ExtractorConfig(roi_side_inter_eye_ratio=0)


if __name__ == "__main__":
    unittest.main()
