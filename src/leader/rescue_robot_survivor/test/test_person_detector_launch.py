"""Static launch and configuration contracts for the survivor detector."""

import importlib.util
from pathlib import Path
from unittest.mock import patch

from launch import LaunchDescription
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch/person_detector.launch.py"
CONFIG_FILE = PACKAGE_ROOT / "config/person_detector.yaml"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "person_detector_launch", LAUNCH_FILE
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_yaml_has_required_defaults():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    parameters = config["/leader/person_detector"]["ros__parameters"]
    assert {
        "image_topic": "/leader/camera/color/image_rect",
        "debug_image_topic": "/leader/survivor/debug_image",
        "model_name": "yolo11n.pt",
        "confidence_threshold": 0.5,
        "device": "auto",
    }.items() <= parameters.items()


def test_launch_is_importable_and_does_not_start_a_camera():
    module = _load_launch_module()
    with patch.object(
        module,
        "get_package_share_directory",
        return_value=str(PACKAGE_ROOT),
    ):
        description = module.generate_launch_description()
    assert isinstance(description, LaunchDescription)
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert "realsense2_camera" not in source
    for name in (
        "image_topic",
        "debug_image_topic",
        "model_name",
        "confidence_threshold",
        "device",
        "aligned_depth_topic",
        "depth_roi_width_ratio",
        "depth_roi_height_ratio",
        "min_depth_m",
        "max_depth_m",
        "min_valid_depth_pixels",
        "depth_scale_m_per_unit",
        "show_depth_roi",
        "sync_queue_size",
        "sync_slop_sec",
        "camera_info_topic",
        "camera_positions_topic",
        "show_camera_xyz",
    ):
        assert f'"{name}"' in source
