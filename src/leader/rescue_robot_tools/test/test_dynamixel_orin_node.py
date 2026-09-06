"""Safety and compatibility tests for raw Dynamixel command routing."""

from types import SimpleNamespace

from std_msgs.msg import Float64MultiArray

from dynamixel_orin_node import DynamixelOrinNode


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
            "rx64_min": 450,
            "rx64_max": 775,
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

    assert harness.calls == [("torque", 33, True), ("position", 33, 520, 450, 775)]


def test_legacy_three_field_command_remains_compatible():
    harness = make_harness()

    DynamixelOrinNode.command_callback(
        harness, Float64MultiArray(data=[-1.0, 450.0, 1.0])
    )

    assert ("torque", 33, True) in harness.calls
    assert ("torque", 2, True) in harness.calls
    assert ("position", 2, 450, 1, 1021) in harness.calls
