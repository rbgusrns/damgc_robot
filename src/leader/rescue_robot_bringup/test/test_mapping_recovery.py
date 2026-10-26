"""Exercise recovery without a robot, ROS clock or wall-clock sleeps."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[4]
spec = importlib.util.spec_from_file_location('mapping_recovery', ROOT/'scripts/mapping_recovery.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
VideoRecovery = module.VideoRecovery
KEYS = ('depth', 'filtered_depth', 'visual_odometry', 'local_odometry', 'map')


def harness():
    monitor = NS(counts=dict.fromkeys(KEYS, 10), video_outage=Mock(return_value='filtered depth gap'),
                 problems=Mock(return_value=[]))
    explorer = NS(paused=False, stopped=False, _pause_actions_settled=Mock(return_value=True))
    def pause(reason):
        explorer.paused = explorer.stopped = True
    def resume():
        explorer.paused = explorer.stopped = False
        return True
    def stop(reason):
        explorer.paused = False
        explorer.stopped = True
    explorer._pause_for_video = Mock(side_effect=pause)
    explorer._resume_after_video = Mock(side_effect=resume)
    explorer._stop = Mock(side_effect=stop)
    return monitor, explorer


def test_outage_requires_new_samples_stable_inputs_and_drained_actions():
    monitor, explorer = harness()
    recovery = VideoRecovery()
    recovery.poll(monitor, explorer, 0)
    assert explorer.paused
    recovery.poll(monitor, explorer, 10)
    explorer._resume_after_video.assert_not_called()
    monitor.counts = dict.fromkeys(KEYS, 13)
    explorer._pause_actions_settled.return_value = False
    recovery.poll(monitor, explorer, 11)
    recovery.poll(monitor, explorer, 14)
    explorer._resume_after_video.assert_not_called()
    explorer._pause_actions_settled.return_value = True
    recovery.poll(monitor, explorer, 15)
    recovery.poll(monitor, explorer, 16.9)
    explorer._resume_after_video.assert_not_called()
    recovery.poll(monitor, explorer, 17)
    explorer._resume_after_video.assert_called_once()
    assert not explorer.stopped
    monitor.problems.assert_called_with(explorer, image_age_limit=2.)


def test_recovery_stability_resets_on_another_gap():
    monitor, explorer = harness()
    recovery = VideoRecovery()
    recovery.poll(monitor, explorer, 0)
    monitor.counts = dict.fromkeys(KEYS, 13)
    recovery.poll(monitor, explorer, 1)
    monitor.problems.return_value = ['depth stale']
    recovery.poll(monitor, explorer, 2)
    monitor.problems.return_value = []
    recovery.poll(monitor, explorer, 3)
    recovery.poll(monitor, explorer, 4)
    explorer._resume_after_video.assert_not_called()
    recovery.poll(monitor, explorer, 5)
    explorer._resume_after_video.assert_called_once()


def test_operator_stop_during_pause_never_resumes():
    monitor, explorer = harness()
    recovery = VideoRecovery()
    recovery.poll(monitor, explorer, 0)
    explorer._stop('operator requested stop')
    monitor.counts = dict.fromkeys(KEYS, 13)
    for now in (1, 3, 60):
        recovery.poll(monitor, explorer, now)
    explorer._resume_after_video.assert_not_called()


def test_unrecovered_inputs_timeout_and_stay_stopped():
    monitor, explorer = harness()
    recovery = VideoRecovery()
    recovery.poll(monitor, explorer, 0)
    recovery.poll(monitor, explorer, 60)
    explorer._stop.assert_called_once()
    monitor.counts = dict.fromkeys(KEYS, 13)
    recovery.poll(monitor, explorer, 70)
    explorer._resume_after_video.assert_not_called()


def test_repeated_outages_are_bounded():
    monitor, explorer = harness()
    recovery = VideoRecovery(max_recoveries=1)
    recovery.poll(monitor, explorer, 0)
    monitor.counts = dict.fromkeys(KEYS, 13)
    recovery.poll(monitor, explorer, 1)
    recovery.poll(monitor, explorer, 3)
    assert not explorer.paused
    recovery.poll(monitor, explorer, 4)
    assert explorer.stopped and not explorer.paused
    explorer._pause_for_video.assert_called_once()


def test_map_or_pose_pause_enters_same_recovery_protocol():
    monitor, explorer = harness()
    recovery = VideoRecovery()
    explorer._pause_for_video('map stale')
    recovery.poll(monitor, explorer, 10)
    assert recovery.started_at == 10
    monitor.counts = dict.fromkeys(KEYS, 13)
    recovery.poll(monitor, explorer, 11)
    recovery.poll(monitor, explorer, 13)
    explorer._resume_after_video.assert_called_once()


def test_solo_recovery_does_not_wait_for_unused_visual_odometry():
    monitor, explorer = harness()
    monitor.required_streams = ('depth', 'filtered_depth', 'local_odometry', 'map')
    recovery = VideoRecovery()
    recovery.poll(monitor, explorer, 0)
    for key in monitor.required_streams:
        monitor.counts[key] += 3
    recovery.poll(monitor, explorer, 1)
    recovery.poll(monitor, explorer, 3)
    explorer._resume_after_video.assert_called_once()
    assert monitor.counts['visual_odometry'] == 10
