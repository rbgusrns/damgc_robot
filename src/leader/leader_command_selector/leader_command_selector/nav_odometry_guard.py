"""Require fresh wheel odometry for Nav2; visual disagreement is not a veto."""

from math import isfinite


class NavOdometryGuard:
    """Check measurement and receipt age independently without comparing VSLAM."""

    def __init__(self, timeout=0.5):
        if not isfinite(timeout) or timeout <= 0:
            raise ValueError("Wheel odometry timeout must be finite and positive")
        self.timeout = timeout
        self.reset()

    def reset(self):
        self.stamp = None
        self.received = None
        self.invalid = False

    def update(self, stamp, received, valid=True):
        if not valid or not all(isfinite(v) for v in (stamp, received)):
            self.invalid = True
            return
        if self.stamp is not None and stamp <= self.stamp:
            if stamp < self.stamp:
                self.reset()
                self.invalid = True
            return
        self.stamp, self.received = stamp, received
        self.invalid = False

    def check(self, now, ros_now):
        if self.invalid:
            return "NAV2_WHEEL_ODOM_INVALID"
        if self.stamp is None:
            return "WAITING_NAV2_WHEEL_ODOMETRY"
        if now-self.received > self.timeout or not -0.1 <= ros_now-self.stamp <= self.timeout:
            return "NAV2_WHEEL_ODOM_STALE"
        return "READY"
