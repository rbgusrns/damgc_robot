"""Static contracts for the integrated Follower AprilTag drive launch."""

import ast
import importlib.util
from pathlib import Path

from launch import LaunchDescription


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
FOLLOWER_ROOT = PACKAGE_ROOT.parent
DRIVE_LAUNCH = PACKAGE_ROOT / "launch/follower_apriltag_drive.launch.py"


def _source(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _string_constants(path: Path) -> list[str]:
    tree = ast.parse(_source(path), filename=str(path))
    return [
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    ]


def test_integrated_launch_is_importable() -> None:
    assert DRIVE_LAUNCH.is_file()
    spec = importlib.util.spec_from_file_location(
        "follower_apriltag_drive_launch", DRIVE_LAUNCH
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert isinstance(module.generate_launch_description(), LaunchDescription)


def test_integrated_launch_declares_bridge_arguments_and_safe_defaults() -> None:
    source = _source(DRIVE_LAUNCH)
    for argument in (
        "use_stm32_bridge",
        "i2c_device",
        "i2c_address",
        "i2c_write_enabled",
    ):
        assert 'DeclareLaunchArgument(\n            "%s"' % argument in source
    assert 'default_value="/dev/i2c-7"' in source
    assert 'default_value="66"' in source
    assert source.count('default_value="true"') == 2
    assert 'condition=IfCondition(use_stm32_bridge)' in source


def test_integrated_launch_reuses_exactly_one_selected_guard() -> None:
    launch_files = [
        value
        for value in _string_constants(DRIVE_LAUNCH)
        if value.endswith(".launch.py")
    ]
    assert launch_files.count("follower_apriltag.launch.py") == 1
    assert launch_files.count("approach_controller.launch.py") == 1
    assert launch_files.count("command_selector.launch.py") == 1
    assert launch_files.count("selected_velocity_guard.launch.py") == 1
    assert launch_files.count("velocity_guard.launch.py") == 0
    assert launch_files.count("stm32_bridge.launch.py") == 1


def test_integrated_startup_overrides_are_explicit() -> None:
    source = _source(DRIVE_LAUNCH)
    assert '{"enabled_on_startup": "true"}' in source
    assert '{"source_mode": "APPROACH"}' in source
    assert '{"guard_enabled_on_startup": "false"}' in source


def test_selected_pipeline_and_bridge_remap_are_explicit() -> None:
    guard_launch = _source(
        FOLLOWER_ROOT / "follower_control/launch/selected_velocity_guard.launch.py"
    )
    source = _source(DRIVE_LAUNCH)
    assert '"command_topic": "/follower/selected_cmd_vel"' in guard_launch
    assert 'SetRemap(src="cmd_vel", dst="/follower/safe_cmd_vel")' in source
    assert '"namespace": "follower"' in source
    assert '"transport": "i2c"' in source


def test_individual_launch_defaults_remain_safe_and_compatible() -> None:
    approach_launch = _source(
        FOLLOWER_ROOT
        / "follower_approach_control/launch/approach_controller.launch.py"
    )
    selector_launch = _source(
        FOLLOWER_ROOT
        / "follower_command_selector/launch/command_selector.launch.py"
    )
    guard_launch = _source(
        FOLLOWER_ROOT / "follower_control/launch/selected_velocity_guard.launch.py"
    )
    assert '"enabled_on_startup",\n                default_value="false"' in approach_launch
    assert '"source_mode",\n                default_value="STOP"' in selector_launch
    assert (
        '"guard_enabled_on_startup",\n                default_value="false"'
        in guard_launch
    )
