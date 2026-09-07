"""Safety and compatibility tests for raw Dynamixel command routing."""

from pathlib import Path
from types import SimpleNamespace

from std_msgs.msg import Float64MultiArray

from dynamixel_orin_node import DynamixelOrinNode


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def make_harness():
    calls = []
    statuses = []
    return SimpleNamespace(
        controller=SimpleNamespace(
            set_torque=lambda device_id, enabled: calls.append(
                ("torque", device_id, enabled)
            ),
            set_position=lambda device_id, position, minimum, maximum: calls.append(
                ("position", device_id, position, minimum, maximum)
            ),
        ),
        profile={
            "rx64_id": 33,
            "rx64_min": 260,
            "rx64_max": 670,
            "rx28_id": 2,
            "rx28_min": 1,
            "rx28_max": 1021,
        },
        calls=calls,
        publish_status=statuses.append,
        statuses=statuses,
    )


def test_targeted_gripper_command_does_not_enable_rx64():
    harness = make_harness()

    DynamixelOrinNode.command_callback(
        harness, Float64MultiArray(data=[-1.0, 1000.0, -1.0, 1.0])
    )

    assert harness.calls == [("torque", 2, True), ("position", 2, 1000, 1, 1021)]


def test_targeted_lift_command_does_not_touch_rx28():
    harness = make_harness()

    DynamixelOrinNode.command_callback(
        harness, Float64MultiArray(data=[520.0, -1.0, 1.0, -1.0])
    )

    assert harness.calls == [("torque", 33, True), ("position", 33, 520, 260, 670)]


def test_legacy_three_field_command_remains_compatible():
    harness = make_harness()

    DynamixelOrinNode.command_callback(
        harness, Float64MultiArray(data=[-1.0, 450.0, 1.0])
    )

    assert ("torque", 33, True) in harness.calls
    assert ("torque", 2, True) in harness.calls
    assert ("position", 2, 450, 1, 1021) in harness.calls


def test_disabled_startup_pose_and_torque_write_nothing():
    harness = make_harness()

    DynamixelOrinNode._apply_startup_configuration(
        harness,
        startup_pose_enabled=False,
        startup_torque=False,
        startup_rx64=600,
        startup_rx28=500,
    )

    assert harness.calls == []
    assert harness.statuses == ["STARTUP pose disabled torque=0"]


def test_leader_compatible_startup_pose_remains_enabled():
    harness = make_harness()

    DynamixelOrinNode._apply_startup_configuration(
        harness,
        startup_pose_enabled=True,
        startup_torque=True,
        startup_rx64=600,
        startup_rx28=500,
    )

    assert harness.calls == [
        ("torque", 33, True),
        ("torque", 2, True),
        ("position", 33, 600, 260, 670),
        ("position", 2, 500, 1, 1021),
    ]


def test_startup_pose_disabled_does_not_validate_unused_positions():
    harness = make_harness()

    DynamixelOrinNode._apply_startup_configuration(
        harness,
        startup_pose_enabled=False,
        startup_torque=False,
        startup_rx64=-1,
        startup_rx28=-1,
    )

    assert harness.calls == []


def test_shared_startup_defaults_remain_leader_compatible_and_typed():
    source = (PACKAGE_ROOT / "launch/dynamixel_orin.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'DeclareLaunchArgument("robot", default_value="leader")' in source
    assert 'DeclareLaunchArgument("startup_torque", default_value="true")' in source
    assert (
        'DeclareLaunchArgument("startup_pose_enabled", default_value="true")'
        in source
    )
    assert 'LaunchConfiguration("startup_torque"), value_type=bool' in source
    assert 'LaunchConfiguration("startup_pose_enabled"), value_type=bool' in source
