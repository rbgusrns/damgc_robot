"""Synthetic camera scenes verify collision geometry and unknown-space handling."""
import importlib.util
from pathlib import Path
import numpy as np

spec = importlib.util.spec_from_file_location(
    'depth_obstacle_scan', Path(__file__).parents[1] / 'scripts/depth_obstacle_scan.py')
scan_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scan_module)
ROTATION = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
TRANSLATION = np.array([0., 0., .13])


def test_nearest_collision_wins_for_multiple_heights_in_one_beam():
    ranges = scan_module.project_scan(np.array([[2.], [1.]]),
        (100., 100., 0., 0.), ROTATION, TRANSLATION, stride=1)
    assert abs(ranges[180]-1.) < 1e-6
    assert np.isfinite(ranges).sum() == 1


def test_unknown_depth_and_unseen_directions_do_not_become_free():
    ranges = scan_module.project_scan(np.array([[0., 1., np.nan, 5.]]),
        (1000., 100., 1., 0.), ROTATION, TRANSLATION, stride=1)
    assert np.isfinite(ranges).sum() == 1
    assert np.isnan(ranges).sum() == 359
    assert not np.isinf(ranges).any()


def test_floor_and_self_returns_are_removed():
    args = (np.array([[1.]]), (100., 100., 0., 0.), ROTATION)
    assert np.isnan(scan_module.project_scan(*args, np.array([0., 0., -1.]))).all()
    assert np.isnan(scan_module.project_scan(np.array([[.3]]), args[1],
                                            ROTATION, TRANSLATION)).all()


def test_mounting_translation_changes_obstacle_distance():
    ranges = scan_module.project_scan(np.array([[1.]]), (100., 100., 0., 0.),
                                      ROTATION, np.array([.2, 0., .13]))
    assert abs(ranges[180]-1.2) < 1e-6
