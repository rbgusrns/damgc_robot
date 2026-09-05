"""Regression tests for the existing Leader gripper sequence."""

from pathlib import Path
from types import SimpleNamespace

from std_msgs.msg import Bool, String

from gripper_sequence_node import GripperSequenceNode, SequenceState


def make_harness() -> SimpleNamespace:
    raw_commands = []
    gripper_commands = []
    statuses = []
    clock = {"now": 10.0}
    harness = SimpleNamespace(
        _enabled=True,
        _state=SequenceState.WAITING_FOR_TAG,
        _tag_detected=False,
        _alignment_state="TAG_LOST",
        _open_raw=1021,
        _close_raw=450,
        _close_wait=2.0,
        _deadline=None,
        _now=lambda: clock["now"],
        _publish_raw=lambda rx64, rx28, torque: raw_commands.append(
            (rx64, rx28, torque)
        ),
        _publish_gripper=gripper_commands.append,
        _publish_status=statuses.append,
    )
    harness.raw_commands = raw_commands
    harness.gripper_commands = gripper_commands
    harness.statuses = statuses
    harness.clock = clock
    return harness


def test_alignment_topic_defaults_to_authoritative_base_state() -> None:
    scripts_dir = Path(__file__).resolve().parents[1] / "scripts"
    launch_file = Path(__file__).resolve().parents[1] / "launch" / "gripper_sequence.launch.py"

    assert '"alignment_topic", "/leader/base_alignment/state"' in (
        scripts_dir / "gripper_sequence_node.py"
    ).read_text(encoding="utf-8")
    assert 'default_value="/leader/base_alignment/state"' in launch_file.read_text(
        encoding="utf-8"
    )


def test_detection_opens_once_and_only_aligned_closes() -> None:
    harness = make_harness()

    GripperSequenceNode._detection_callback(harness, Bool(data=True))
    GripperSequenceNode._detection_callback(harness, Bool(data=True))

    assert harness._state == SequenceState.OPENING
    assert harness.raw_commands == [(-1.0, 1021.0, 1.0)]

    for state in ("COARSE", "NEAR", "FINAL_YAW_ALIGN", "FINAL_APPROACH", "STABILIZING"):
        GripperSequenceNode._alignment_callback(harness, String(data=state))
        GripperSequenceNode._on_timer(harness)
        assert harness._state == SequenceState.OPENING

    GripperSequenceNode._alignment_callback(harness, String(data="ALIGNED"))
    GripperSequenceNode._on_timer(harness)

    assert harness._state == SequenceState.CLOSING
    assert harness.raw_commands[-1] == (-1.0, 450.0, 1.0)
    assert harness._deadline == 12.0


def test_close_wait_lifts_once_despite_repeated_aligned() -> None:
    harness = make_harness()
    harness._state = SequenceState.CLOSING
    harness._tag_detected = True
    harness._alignment_state = "ALIGNED"
    harness._deadline = 12.0

    harness.clock["now"] = 12.0
    GripperSequenceNode._on_timer(harness)
    GripperSequenceNode._alignment_callback(harness, String(data="ALIGNED"))
    GripperSequenceNode._on_timer(harness)

    assert harness._state == SequenceState.LIFTING
    assert harness.gripper_commands == ["rx64_middle"]

    harness.clock["now"] = 14.0
    GripperSequenceNode._on_timer(harness)
    GripperSequenceNode._on_timer(harness)

    assert harness._state == SequenceState.DONE
    assert harness.gripper_commands == ["rx64_middle"]


def test_done_does_not_rearm_on_detection_flicker_while_aligned_latched() -> None:
    harness = make_harness()
    harness._state = SequenceState.DONE
    harness._alignment_state = "ALIGNED"

    GripperSequenceNode._detection_callback(harness, Bool(data=False))
    GripperSequenceNode._on_timer(harness)
    GripperSequenceNode._detection_callback(harness, Bool(data=True))
    GripperSequenceNode._on_timer(harness)

    assert harness._state == SequenceState.DONE
    assert not harness.raw_commands
    assert not harness.gripper_commands


def test_done_rearms_only_after_detection_and_alignment_clear() -> None:
    harness = make_harness()
    harness._state = SequenceState.DONE
    harness._tag_detected = False
    harness._alignment_state = "TAG_LOST"

    GripperSequenceNode._on_timer(harness)

    assert harness._state == SequenceState.WAITING_FOR_TAG
    assert harness.statuses[-1] == "WAITING_FOR_TAG next cycle armed"
