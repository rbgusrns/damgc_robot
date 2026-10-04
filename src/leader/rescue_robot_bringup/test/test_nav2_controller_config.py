"""Configuration contract for the Leader's Nav2 path-tracking controller."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT / "config/nvblox_nav2.yaml"
PACKAGE_XML = PACKAGE_ROOT / "package.xml"


def test_rpp_controller_and_goal_tolerances_are_configured():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    controller = config["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]

    assert controller["controller_plugins"] == ["FollowPath"]
    assert controller["general_goal_checker"]["xy_goal_tolerance"] == 0.05
    assert controller["progress_checker"]["required_movement_radius"] == 0.05
    assert (
        follow_path["plugin"]
        == "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    )
    assert follow_path["desired_linear_vel"] == 0.10
    assert follow_path["min_lookahead_dist"] < follow_path["max_lookahead_dist"]
    assert follow_path["use_velocity_scaled_lookahead_dist"] is True
    assert follow_path["use_collision_detection"] is True


def test_bringup_declares_the_rpp_runtime_dependency():
    package = ET.parse(PACKAGE_XML).getroot()
    dependencies = {node.text for node in package.findall("exec_depend")}

    assert "nav2_regulated_pure_pursuit_controller" in dependencies
    assert "nav2_dwb_controller" not in dependencies
