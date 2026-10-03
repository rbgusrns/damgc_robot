"""Keep DWB's near-goal behavior aligned with the action goal checker."""

from pathlib import Path
import re


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_goal_checker_and_dwb_use_matching_xy_tolerance() -> None:
    config = (PACKAGE_ROOT / "config/nvblox_nav2.yaml").read_text(encoding="utf-8")
    checker = re.search(
        r"general_goal_checker:\s*\n(?:\s+.*\n)*?\s+xy_goal_tolerance: ([0-9.]+)",
        config,
    )
    dwb = re.search(
        r"FollowPath:\s*\n(?:\s+.*\n)*?\s+xy_goal_tolerance: ([0-9.]+)",
        config,
    )
    assert checker is not None
    assert dwb is not None
    assert checker.group(1) == "0.05"
    assert dwb.group(1) == checker.group(1)
