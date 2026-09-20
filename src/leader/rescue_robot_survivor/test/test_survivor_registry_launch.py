"""Registry launch and configuration contracts."""

import importlib.util
from pathlib import Path
from unittest.mock import patch

from launch import LaunchDescription
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch/survivor_registry.launch.py"
CONFIG_FILE = PACKAGE_ROOT / "config/survivor_registry.yaml"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "survivor_registry_launch", LAUNCH_FILE
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_yaml_has_complete_registry_defaults():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    parameters = config["/leader/survivor_registry"]["ros__parameters"]
    assert parameters == {
        "input_topic": "/leader/survivor/map_positions",
        "tracks_topic": "/leader/survivor/tracks",
        "reset_service": "/leader/survivor/registry/reset",
        "map_frame": "map",
        "association_radius_m": 0.50,
        "reassociation_radius_m": 0.75,
        "confirm_min_duration_sec": 2.0,
        "confirm_min_hits": 4,
        "tentative_max_gap_sec": 0.8,
        "visible_timeout_sec": 4.0,
        "position_ema_alpha": 0.50,
        "registry_publish_hz": 2.0,
    }
    assert config["/leader/survivor_registry_visualizer"][
        "ros__parameters"
    ] == {
        "input_topic": "/leader/survivor/tracks",
        "output_topic": "/leader/survivor/registry_markers",
        "map_frame": "map",
        "marker_namespace": "survivor_registry",
        "marker_scale": 0.20,
        "text_height": 0.18,
        "text_z_offset": 0.30,
    }


def test_launch_starts_only_registry_and_exposes_every_parameter():
    module = _load_launch_module()
    with patch.object(
        module,
        "get_package_share_directory",
        return_value=str(PACKAGE_ROOT),
    ):
        description = module.generate_launch_description()
    assert isinstance(description, LaunchDescription)
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    for forbidden in (
        "realsense2_camera",
        "visual_slam",
        "nvblox",
        "person_detector_node",
    ):
        assert forbidden not in source
    assert 'executable="survivor_registry_node"' in source
    assert 'executable="survivor_registry_visualizer_node"' in source
    for name in yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))[
        "/leader/survivor_registry"
    ]["ros__parameters"]:
        assert name in source or "for name, value in defaults.items()" in source
