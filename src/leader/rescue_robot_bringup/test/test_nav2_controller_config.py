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


def test_recovered_cost_field_matches_controller_distance_estimate():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    controller = config['controller_server']['ros__parameters']['FollowPath']
    for name in ('local_costmap', 'global_costmap'):
        inflation = config[name][name]['ros__parameters']['inflation_layer']
        assert inflation['cost_scaling_factor'] == controller['inflation_cost_scaling_factor']
        # The recovered profile uses the pre-autonomy 0.3 m inflation field.
        # The controller must estimate distances using that same field.
        assert inflation['inflation_radius'] >= controller['cost_scaling_dist']
        assert inflation['inflation_radius'] == .30
    assert controller['use_collision_detection'] is True


def test_automatic_steps_split_position_and_heading_and_replan():
    config = yaml.safe_load(CONFIG.read_text())
    controller = config['controller_server']['ros__parameters']
    checker = controller['position_goal_checker']
    assert checker['xy_goal_tolerance'] == .12
    assert checker['yaw_goal_tolerance'] >= 3.14159
    assert controller['general_goal_checker']['yaw_goal_tolerance'] == .25
    for filename, checker_id in [('navigate_to_pose_steps.xml', 'position_goal_checker'),
                                  ('navigate_to_pose_nvblox.xml', 'general_goal_checker')]:
        tree = ET.parse(CONFIG.parent/filename).getroot()
        assert tree.find('.//PipelineSequence') is not None
        assert float(tree.find('.//RateController').attrib['hz']) == .5
        assert tree.find('.//FollowPath').attrib['goal_checker_id'] == checker_id
    plugins = config['bt_navigator']['ros__parameters']['plugin_lib_names']
    assert 'nav2_rate_controller_bt_node' in plugins
    assert 'nav2_pipeline_sequence_bt_node' in plugins
    assert 'backup' in config['behavior_server']['ros__parameters']['behavior_plugins']
