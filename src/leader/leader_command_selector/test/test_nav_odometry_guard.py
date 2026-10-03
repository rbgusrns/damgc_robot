"""Wheel odometry freshness is independent of VSLAM availability or agreement."""

from leader_command_selector.nav_odometry_guard import NavOdometryGuard


def test_wheel_only_is_ready_without_visual_samples_or_warmup():
    guard = NavOdometryGuard()
    assert guard.check(10, 10) == "WAITING_NAV2_WHEEL_ODOMETRY"
    guard.update(10, 10)
    assert guard.check(10, 10) == "READY"


def test_duplicate_old_future_and_invalid_samples_block_until_fresh():
    guard = NavOdometryGuard()
    guard.update(10, 10)
    guard.update(10, 11)
    assert guard.check(11, 11) == "NAV2_WHEEL_ODOM_STALE"
    guard.update(11, 11)
    assert guard.check(11, 11) == "READY"
    guard.update(12, 11)
    assert guard.check(11, 11) == "NAV2_WHEEL_ODOM_STALE"
    guard.update(13, 13, valid=False)
    assert guard.check(13, 13) == "NAV2_WHEEL_ODOM_INVALID"
    guard.update(13, 13)
    assert guard.check(13, 13) == "READY"


def test_backward_sensor_clock_requires_new_sample():
    guard = NavOdometryGuard()
    guard.update(10, 10)
    guard.update(9, 11)
    assert guard.check(11, 11) == "NAV2_WHEEL_ODOM_INVALID"
    guard.update(11, 11)
    assert guard.check(11, 11) == "READY"
