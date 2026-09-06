"""Static contracts for the role-specific cooperative-drive launch files."""

import importlib.util
from pathlib import Path
from unittest.mock import patch

from launch import LaunchDescription


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
FOLLOWER_LAUNCH = (
    REPOSITORY_ROOT
    / "src/follower/follower_supply_perception/launch"
    / "follower_cooperation_drive.launch.py"
)
LEADER_LAUNCH = (
    REPOSITORY_ROOT
    / "src/leader/rescue_robot_bringup/launch/leader_cooperation_drive.launch.py"
)
RUNNER = REPOSITORY_ROOT / "scripts/run_cooperative_transport.sh"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_role_launch_files_are_importable() -> None:
    follower = _load("follower_cooperation_drive", FOLLOWER_LAUNCH)
    leader = _load("leader_cooperation_drive", LEADER_LAUNCH)
    assert isinstance(follower.generate_launch_description(), LaunchDescription)
    with patch.object(
        leader,
        "get_package_share_directory",
        side_effect=lambda package: f"/tmp/share/{package}",
    ):
        assert isinstance(leader.generate_launch_description(), LaunchDescription)


def test_follower_path_is_fail_closed_and_uses_cooperation_source() -> None:
    source = FOLLOWER_LAUNCH.read_text(encoding="utf-8")
    assert 'default_value="false"' in source
    assert '{"source_mode": "COOPERATION"}' in source
    assert 'SetRemap(src="cmd_vel", dst="/follower/safe_cmd_vel")' in source
    assert source.count('"selected_velocity_guard.launch.py"') == 1
    assert source.count('"stm32_bridge.launch.py"') == 1


def test_leader_reuses_coordinator_and_motor_bridge() -> None:
    source = LEADER_LAUNCH.read_text(encoding="utf-8")
    assert source.count('"leader_cooperation.launch.py"') == 1
    assert source.count('"stm32_bridge.launch.py"') == 1
    assert '"namespace": "leader"' in source


def test_runner_requires_follower_heartbeat_before_teleop() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    heartbeat = source.index("ros2 topic echo /follower/status")
    enable = source.index("ros2 service call /cooperation/enable", heartbeat)
    teleop = source.index("arrow_key_teleop.py", enable)
    assert heartbeat < enable < teleop


def test_runner_keeps_follower_guard_disabled_on_startup() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    assert "guard_enabled_on_startup:=false" in source
    assert "COOP_USE_STM32_BRIDGE" in source
