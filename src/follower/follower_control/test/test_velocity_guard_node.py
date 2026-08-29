"""ROS message-boundary and legacy-status tests for the Follower guard."""

from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

import follower_control.velocity_guard_node as node_module
from follower_control.velocity_guard_logic import GuardParameters, PlanarVelocity
from follower_control.velocity_guard_node import VelocityGuardNode


class RecordingPublisher:
    """Publisher double that records every outgoing message."""

    def __init__(self) -> None:
        self.messages = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class RecordingLogger:
    """Logger double accepting normal and throttled calls."""

    def __init__(self) -> None:
        self.messages = []

    def info(self, message: str, **_kwargs: object) -> None:
        self.messages.append(message)

    def warning(self, message: str, **_kwargs: object) -> None:
        self.messages.append(message)


def parameters() -> GuardParameters:
    """Return deterministic guard parameters."""
    return GuardParameters(0.25, 0.8, 0.25, 0.8, 0.3, 0.1, 1.0e-9, False)


def make_twist(linear_x: float = 0.1, angular_z: float = 0.2) -> Twist:
    """Create a planar command."""
    message = Twist()
    message.linear.x = linear_x
    message.angular.z = angular_z
    return message


def make_harness(*, enabled: bool = False) -> SimpleNamespace:
    """Create the state required by callbacks without starting ROS."""
    harness = SimpleNamespace(
        _enabled=enabled,
        _guard_parameters=parameters(),
        _last_command=None,
        _last_received_seconds=None,
        _last_output=PlanarVelocity(),
        _last_output_seconds=0.0,
        _connected=None,
        _cmd_pub=RecordingPublisher(),
        _connected_pub=RecordingPublisher(),
        _status_pub=RecordingPublisher(),
        _shutdown_stop_count=3,
        _logger=RecordingLogger(),
    )
    harness.get_logger = lambda: harness._logger
    harness._force_zero = lambda now: VelocityGuardNode._force_zero(harness, now)
    harness._publish_health = lambda fresh: VelocityGuardNode._publish_health(
        harness, fresh
    )
    harness._to_twist = VelocityGuardNode._to_twist
    return harness


def assert_zero(message: Twist) -> None:
    """Assert all Twist axes are zero."""
    assert message.linear.x == message.linear.y == message.linear.z == 0.0
    assert message.angular.x == message.angular.y == message.angular.z == 0.0


def test_output_twist_uses_only_planar_axes() -> None:
    message = VelocityGuardNode._to_twist(PlanarVelocity(0.2, -0.4))
    assert message.linear.x == 0.2
    assert message.angular.z == -0.4
    assert message.linear.y == message.linear.z == 0.0
    assert message.angular.x == message.angular.y == 0.0


def test_invalid_command_immediately_clears_cache_and_publishes_zero(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(node_module.time, "monotonic", lambda: 10.0)
    harness = make_harness(enabled=True)
    harness._last_command = PlanarVelocity(0.1, 0.2)
    harness._last_received_seconds = 9.9
    message = make_twist()
    message.angular.x = 0.01

    VelocityGuardNode._on_command(harness, message)

    assert harness._last_command is None
    assert harness._last_received_seconds is None
    assert_zero(harness._cmd_pub.messages[-1])


def test_enable_transition_publishes_zero_and_requires_fresh_command(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(node_module.time, "monotonic", lambda: 20.0)
    harness = make_harness(enabled=False)
    harness._last_command = PlanarVelocity(0.1, 0.2)
    harness._last_received_seconds = 19.9

    response = VelocityGuardNode._on_enable(
        harness, SetBool.Request(data=True), SetBool.Response()
    )

    assert response.success is True
    assert "fresh upstream command" in response.message
    assert harness._enabled is True
    assert harness._last_command is None
    assert harness._last_received_seconds is None
    assert_zero(harness._cmd_pub.messages[-1])


def test_disabled_with_fresh_input_outputs_zero_but_preserves_active_status(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(node_module.time, "monotonic", lambda: 30.0)
    harness = make_harness(enabled=False)
    harness._last_command = PlanarVelocity(0.1, 0.2)
    harness._last_received_seconds = 29.9

    VelocityGuardNode._on_timer(harness)

    assert_zero(harness._cmd_pub.messages[-1])
    assert harness._connected_pub.messages[-1].data is True
    assert harness._status_pub.messages[-1].data == "ACTIVE"


def test_enabled_fresh_command_is_slew_limited(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(node_module.time, "monotonic", lambda: 40.02)
    harness = make_harness(enabled=True)
    harness._last_command = PlanarVelocity(0.25, 0.8)
    harness._last_received_seconds = 40.0
    harness._last_output_seconds = 40.0

    VelocityGuardNode._on_timer(harness)

    output = harness._cmd_pub.messages[-1]
    assert output.linear.x == pytest.approx(0.005)
    assert output.angular.z == pytest.approx(0.016)
    assert harness._connected_pub.messages[-1].data is True
    assert harness._status_pub.messages[-1].data == "ACTIVE"


def test_timeout_immediately_resets_output_and_reports_ready(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(node_module.time, "monotonic", lambda: 50.31)
    harness = make_harness(enabled=True)
    harness._last_command = PlanarVelocity(0.1, 0.2)
    harness._last_received_seconds = 50.0
    harness._last_output = PlanarVelocity(0.05, 0.1)
    harness._connected = True

    VelocityGuardNode._on_timer(harness)

    assert_zero(harness._cmd_pub.messages[-1])
    assert harness._connected_pub.messages[-1].data is False
    assert harness._status_pub.messages[-1].data == "READY"


def test_backward_or_zero_dt_fails_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(node_module.time, "monotonic", lambda: 59.9)
    harness = make_harness(enabled=True)
    harness._last_command = PlanarVelocity(0.1, 0.2)
    harness._last_received_seconds = 59.9
    harness._last_output_seconds = 60.0

    VelocityGuardNode._on_timer(harness)

    assert_zero(harness._cmd_pub.messages[-1])


def test_status_heartbeat_is_periodic_but_connection_only_changes_on_edge() -> None:
    harness = make_harness()
    VelocityGuardNode._publish_health(harness, False)
    VelocityGuardNode._publish_health(harness, False)
    assert len(harness._connected_pub.messages) == 1
    assert harness._connected_pub.messages[0].data is False
    assert [message.data for message in harness._status_pub.messages] == [
        "READY",
        "READY",
    ]


def test_shutdown_publishes_configured_zero_burst() -> None:
    harness = make_harness()
    VelocityGuardNode.stop(harness)
    assert len(harness._cmd_pub.messages) == 3
    for message in harness._cmd_pub.messages:
        assert_zero(message)
