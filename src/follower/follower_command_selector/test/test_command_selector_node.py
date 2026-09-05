"""ROS-message and source-switch tests for the command selector node."""

from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter

import follower_command_selector.command_selector_node as node_module
from follower_command_selector.command_selector_logic import (
    CommandSource,
    PlanarCommand,
    SelectorParameters,
)
from follower_command_selector.command_selector_node import CommandSelectorNode


class RecordingPublisher:
    """Publisher double that records messages."""

    def __init__(self) -> None:
        self.messages = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


def make_twist(linear_x: float = 0.1, angular_z: float = 0.2) -> Twist:
    """Create a finite differential-drive command."""
    message = Twist()
    message.linear.x = linear_x
    message.angular.z = angular_z
    return message


def test_twist_output_populates_only_planar_axes() -> None:
    message = CommandSelectorNode._to_twist(PlanarCommand(0.3, -0.4))
    assert message.linear.x == 0.3
    assert message.angular.z == -0.4
    assert message.linear.y == message.linear.z == 0.0
    assert message.angular.x == message.angular.y == 0.0


@pytest.mark.parametrize(
    ("source", "callback"),
    [
        (CommandSource.STOP, CommandSelectorNode._on_approach_command),
        (CommandSource.COOPERATION, CommandSelectorNode._on_approach_command),
        (CommandSource.STOP, CommandSelectorNode._on_cooperation_command),
        (CommandSource.APPROACH, CommandSelectorNode._on_cooperation_command),
    ],
)
def test_unselected_source_callback_is_completely_ignored(source, callback) -> None:
    harness = SimpleNamespace(_source=source)
    callback(harness, make_twist())
    assert vars(harness) == {"_source": source}


def selected_harness(source: CommandSource) -> SimpleNamespace:
    """Create attributes used by one selected-source callback."""
    publisher = RecordingPublisher()
    return SimpleNamespace(
        _source=source,
        _selector_parameters=SelectorParameters(0.35, 0.50, 1.0e-9),
        _approach_command=None,
        _approach_received_seconds=None,
        _cooperation_command=None,
        _cooperation_received_seconds=None,
        _selected_pub=publisher,
        _sanitize=lambda message: CommandSelectorNode._sanitize(
            SimpleNamespace(
                _selector_parameters=SelectorParameters(0.35, 0.50, 1.0e-9)
            ),
            message,
        ),
    )


def test_selected_approach_command_is_cached_with_receipt_time(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(node_module.time, "monotonic", lambda: 20.0)
    harness = selected_harness(CommandSource.APPROACH)
    CommandSelectorNode._on_approach_command(harness, make_twist(0.1, 0.2))
    assert harness._approach_command == PlanarCommand(0.1, 0.2)
    assert harness._approach_received_seconds == 20.0


def test_selected_cooperation_command_is_cached_with_receipt_time(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(node_module.time, "monotonic", lambda: 30.0)
    harness = selected_harness(CommandSource.COOPERATION)
    CommandSelectorNode._on_cooperation_command(harness, make_twist(0.3, -0.4))
    assert harness._cooperation_command == PlanarCommand(0.3, -0.4)
    assert harness._cooperation_received_seconds == 30.0


def test_invalid_selected_command_clears_cache_and_publishes_zero() -> None:
    harness = selected_harness(CommandSource.APPROACH)
    message = make_twist()
    message.linear.x = float("nan")
    CommandSelectorNode._on_approach_command(harness, message)
    assert harness._approach_command is None
    assert harness._approach_received_seconds is None
    assert len(harness._selected_pub.messages) == 1
    assert harness._selected_pub.messages[0].linear.x == 0.0


def parameter_harness() -> SimpleNamespace:
    """Create attributes used by runtime source parameter changes."""
    publisher = RecordingPublisher()
    clears = []
    return SimpleNamespace(
        _source=CommandSource.STOP,
        _clear_commands=lambda: clears.append(True),
        _selected_pub=publisher,
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
        clears=clears,
    )


def test_source_change_publishes_zero_and_requires_fresh_command() -> None:
    harness = parameter_harness()
    result = CommandSelectorNode._on_parameter_change(
        harness, [Parameter("source_mode", value="APPROACH")]
    )
    assert result.successful is True
    assert harness._source == CommandSource.APPROACH
    assert harness.clears == [True]
    assert len(harness._selected_pub.messages) == 1
    assert harness._selected_pub.messages[0].linear.x == 0.0
    assert harness._selected_pub.messages[0].angular.z == 0.0


@pytest.mark.parametrize("value", ["AUTO", "approach", 1])
def test_invalid_source_parameter_is_rejected(value) -> None:
    harness = parameter_harness()
    result = CommandSelectorNode._on_parameter_change(
        harness, [Parameter("source_mode", value=value)]
    )
    assert result.successful is False
    assert harness._source == CommandSource.STOP
    assert harness.clears == []


def test_non_source_runtime_parameter_change_is_rejected() -> None:
    harness = parameter_harness()
    result = CommandSelectorNode._on_parameter_change(
        harness, [Parameter("approach_timeout", value=1.0)]
    )
    assert result.successful is False
