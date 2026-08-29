"""Cross-package configuration contract tests."""

import re
from pathlib import Path

import pytest


def yaml_scalar(path: Path, key: str) -> float:
    """Read one numeric scalar from the small ROS parameter YAML files."""
    pattern = re.compile(r"^\s*%s:\s*([0-9.]+)\s*$" % re.escape(key))
    for line in path.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if match:
            return float(match.group(1))
    raise AssertionError("Missing parameter %s in %s" % (key, path))


def test_controller_and_base_state_targets_match_leader_distance() -> None:
    follower_root = Path(__file__).resolve().parents[2]
    controller_config = (
        follower_root / "follower_approach_control/config/approach_controller.yaml"
    )
    perception_config = (
        follower_root / "follower_supply_perception/config/approach.yaml"
    )
    leader_config = (
        follower_root.parent
        / "leader/leader_approach_control/config/approach_controller.yaml"
    )
    controller_target = yaml_scalar(controller_config, "target_forward")
    perception_target = yaml_scalar(perception_config, "base_target_forward")
    leader_target = yaml_scalar(leader_config, "target_forward")
    assert controller_target == pytest.approx(0.25)
    assert perception_target == pytest.approx(0.25)
    assert controller_target == perception_target == leader_target


def test_live_camera_mode_and_controller_freshness_contract() -> None:
    """Keep the camera mode and timeout validated on the Follower computer."""
    follower_root = Path(__file__).resolve().parents[2]
    controller_config = (
        follower_root / "follower_approach_control/config/approach_controller.yaml"
    )
    camera_launch = (
        follower_root
        / "follower_supply_perception/launch/follower_apriltag.launch.py"
    ).read_text(encoding="utf-8")

    assert yaml_scalar(controller_config, "pose_timeout") == pytest.approx(1.20)
    assert '"framerate": 30.0' in camera_launch
    assert '"io_method": "mmap"' in camera_launch
    assert '"pixel_format": "yuyv"' in camera_launch
