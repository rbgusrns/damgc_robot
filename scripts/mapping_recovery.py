"""Automatic recovery from video reception gaps; no ROS imports or side effects."""


class VideoRecovery:
    """Pause on an outage, drain actions and require sustained fresh inputs."""

    def __init__(self, timeout_sec=60.0, stable_sec=2.0, max_recoveries=3):
        self.timeout_sec = timeout_sec
        self.stable_sec = stable_sec
        self.max_recoveries = max_recoveries
        self.attempts = 0
        self.started_at = None
        self.stable_since = None
        self.counts_at_pause = {}

    def poll(self, monitor, explorer, now):
        if not getattr(explorer, 'paused', False):
            self.started_at = None
            self.stable_since = None
            if explorer.stopped:
                return
            outage = monitor.video_outage()
            if not outage:
                return
            if self.attempts >= self.max_recoveries:
                explorer._stop('Video outage recovery limit reached')
                return
            self.attempts += 1
            self.started_at = now
            self.counts_at_pause = dict(monitor.counts)
            explorer._pause_for_video(outage)
            return

        # The explorer can also pause when its map or pose goes stale first.
        if self.started_at is None:
            if self.attempts >= self.max_recoveries:
                explorer._stop('Video outage recovery limit reached')
                return
            self.attempts += 1
            self.started_at = now
            self.counts_at_pause = dict(monitor.counts)
        # Operator stop or action/selector failures clear paused and cannot resume.
        if now - self.started_at >= self.timeout_sec:
            explorer._stop(f'Live inputs/actions did not recover within {self.timeout_sec:g} seconds')
            return
        fresh_samples = all(monitor.counts[key] - self.counts_at_pause.get(key, 0) >= 3
                            for key in getattr(monitor, 'required_streams',
                                ('depth', 'filtered_depth', 'visual_odometry', 'local_odometry', 'map')))
        problems = monitor.problems(explorer, image_age_limit=2.0)
        if problems or not fresh_samples or not explorer._pause_actions_settled():
            self.stable_since = None
            return
        if self.stable_since is None:
            self.stable_since = now
        elif now - self.stable_since >= self.stable_sec:
            if explorer._resume_after_video():
                self.started_at = self.stable_since = None
