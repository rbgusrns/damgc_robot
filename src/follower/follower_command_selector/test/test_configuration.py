"""Selector package configuration contract tests."""

from pathlib import Path


def test_selector_starts_in_stop_and_owns_distinct_output_topic() -> None:
    package_root = Path(__file__).resolve().parents[1]
    config = (package_root / "config/command_selector.yaml").read_text(
        encoding="utf-8"
    )
    node = (
        package_root
        / "follower_command_selector/command_selector_node.py"
    ).read_text(encoding="utf-8")
    assert "source_mode: STOP" in config
    assert '"approach/cmd_vel_raw"' in node
    assert '"cmd_vel"' in node
    assert '"selected_cmd_vel"' in node
    assert 'create_publisher(Twist, "cmd_vel"' not in node
