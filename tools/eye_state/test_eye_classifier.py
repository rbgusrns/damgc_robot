"""Tests for checkpoint class mapping and independent eye inference."""

import unittest

import numpy as np

from eye_classifier import (
    EyePrediction,
    classify_valid_eye_rois,
    normalize_model_names,
    prediction_from_probabilities,
    prepare_eye_crop,
)


class FakeEye:
    def __init__(self, side, valid, image):
        self.side = side
        self.valid = valid
        self.image = image


class FakeFace:
    def __init__(self, left, right):
        self.left_eye = left
        self.right_eye = right


class FakeClassifier:
    def __init__(self):
        self.received = None

    def predict(self, crops):
        self.received = list(crops)
        return [EyePrediction("OPEN", "open", 0.94,
                              {"OPEN": 0.94, "CLOSED": 0.06})
                for _ in crops]


class FakeProbabilities:
    def __init__(self, values):
        self.data = self
        self.values = values

    def detach(self):
        return self

    def cpu(self):
        return self

    def tolist(self):
        return self.values


class FakeResult:
    def __init__(self, values):
        self.probs = FakeProbabilities(values)


class FakeYOLOModel:
    task = "classify"
    names = {0: "closed", 1: "open"}

    def __init__(self):
        self.calls = 0

    def predict(self, **kwargs):
        self.calls += 1
        return [FakeResult([0.02, 0.98]) for _ in kwargs["source"]]


class EyeClassifierTests(unittest.TestCase):
    def test_names_follow_checkpoint_order_without_index_assumptions(self):
        names = normalize_model_names({0: "closed", 1: "open"})
        result = prediction_from_probabilities(names, [0.03, 0.97], 0.60)
        self.assertEqual(names, {0: "closed", 1: "open"})
        self.assertEqual(result.state, "OPEN")
        self.assertEqual(result.predicted_class, "open")
        self.assertAlmostEqual(result.confidence, 0.97)

    def test_low_confidence_is_not_forced_into_open_or_closed(self):
        names = normalize_model_names(["closed", "open"])
        result = prediction_from_probabilities(names, [0.52, 0.48], 0.60)
        self.assertEqual(result.state, "LOW_CONFIDENCE")
        self.assertEqual(result.predicted_class, "closed")

    def test_invalid_class_mapping_fails(self):
        with self.assertRaisesRegex(ValueError, "model classes"):
            normalize_model_names({0: "awake", 1: "closed"})

    def test_gray_mode_replicates_one_channel_three_times(self):
        crop = np.zeros((12, 16, 3), dtype=np.uint8)
        crop[:, :, 0] = 255
        prepared = prepare_eye_crop(crop, "gray")
        self.assertEqual(prepared.shape, crop.shape)
        self.assertTrue(np.array_equal(prepared[:, :, 0], prepared[:, :, 1]))
        self.assertTrue(np.array_equal(prepared[:, :, 1], prepared[:, :, 2]))

    def test_rgb_mode_preserves_color_crop(self):
        crop = np.zeros((12, 16, 3), dtype=np.uint8)
        crop[:, :, 2] = 255
        prepared = prepare_eye_crop(crop, "rgb")
        np.testing.assert_array_equal(prepared, crop)

    def test_only_valid_eye_rois_are_passed_to_classifier(self):
        left_crop = np.full((24, 24, 3), 1, dtype=np.uint8)
        right_crop = np.full((24, 24, 3), 2, dtype=np.uint8)
        face = FakeFace(FakeEye("left", True, left_crop),
                        FakeEye("right", False, None))
        classifier = FakeClassifier()
        predictions = classify_valid_eye_rois([face], classifier)
        self.assertEqual(list(predictions), [(0, "left")])
        self.assertEqual(len(classifier.received), 1)
        self.assertIs(classifier.received[0], left_crop)

    def test_all_invalid_rois_do_not_call_classifier(self):
        face = FakeFace(FakeEye("left", False, None),
                        FakeEye("right", False, None))
        classifier = FakeClassifier()
        self.assertEqual(classify_valid_eye_rois([face], classifier), {})
        self.assertIsNone(classifier.received)

    def test_two_eyes_keep_separate_prediction_keys(self):
        left = FakeEye("left", True, np.zeros((24, 24, 3), dtype=np.uint8))
        right = FakeEye("right", True, np.ones((24, 24, 3), dtype=np.uint8))
        predictions = classify_valid_eye_rois(
            [FakeFace(left, right)], FakeClassifier()
        )
        self.assertEqual(set(predictions), {(0, "left"), (0, "right")})

    def test_loaded_model_is_reused_and_runtime_names_map_each_prediction(self):
        from eye_classifier import EyeStateClassifier

        classifier = EyeStateClassifier.__new__(EyeStateClassifier)
        classifier.model = FakeYOLOModel()
        classifier.names = normalize_model_names(classifier.model.names)
        classifier.imgsz = 224
        classifier.device = "cpu"
        classifier.preprocessing_mode = "gray"
        classifier.min_confidence = 0.60
        crop = np.zeros((24, 24, 3), dtype=np.uint8)
        first = classifier.predict([crop])[0]
        second = classifier.predict([crop])[0]
        self.assertEqual((first.state, first.predicted_class), ("OPEN", "open"))
        self.assertEqual((second.state, second.predicted_class), ("OPEN", "open"))
        self.assertEqual(classifier.model.calls, 2)


if __name__ == "__main__":
    unittest.main()
