"""Static contracts for the integrated Leader AprilTag drive launch."""

import importlib.util
from pathlib import Path
from unittest.mock import patch

from launch import LaunchDescription


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch/leader_apriltag_drive.launch.py"


def _load():
    spec = importlib.util.spec_from_file_location("leader_drive_launch", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_integrated_launch_is_importable():
    module = _load()
    with patch.object(module, "get_package_share_directory", return_value="/tmp/share"):
        assert isinstance(module.generate_launch_description(), LaunchDescription)


def test_integrated_launch_exposes_safe_gripper_and_lift_defaults():
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("gripper_enabled", default_value="true")' in source
    assert 'DeclareLaunchArgument("gripper_open_raw", default_value="1000")' in source
    assert 'DeclareLaunchArgument("lift_enabled", default_value="false")' in source
    assert 'DeclareLaunchArgument("lift_raw", default_value="-1")' in source
    assert 'condition=IfCondition(LaunchConfiguration("gripper_enabled"))' in source
    assert source.count('"dynamixel_orin.launch.py"') == 1
    assert source.count('"gripper_sequence.launch.py"') == 1
    assert '"alignment_topic": "/leader/base_alignment/state"' in source


def test_existing_drive_startup_safety_is_unchanged():
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert '"guard_enabled_on_startup": "false"' in source
    assert '"controller_enabled_on_startup": "true"' in source
    assert '"transport": "i2c"' in source
