"""Tests for tying an explicit Nav2 goal to the command selector lease."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from action_msgs.msg import GoalStatus, GoalStatusArray


SCRIPT = Path(__file__).parents[1] / "scripts" / "frontier_explorer.py"
SPEC = importlib.util.spec_from_file_location("frontier_explorer", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
FrontierExplorer = MODULE.FrontierExplorer


def goal_status(first_uuid_byte, status):
    entry = GoalStatus()
    entry.goal_info.goal_id.uuid = [first_uuid_byte] + [0] * 15
    entry.status = status
    return entry


def status_message(*entries):
    message = GoalStatusArray()
    message.status_list = list(entries)
    return message


def test_new_nav2_goal_enables_selector_and_terminal_status_clears_lease():
    requests = []
    gate = SimpleNamespace(
        _goal_status_initialized=False,
        _seen_nav2_goal_ids=set(),
        _controlled_nav2_goal_ids=set(),
        _nav2_goal_active=False,
        _selector_enable_requested=False,
        enable_selector_on_nav2_goal=True,
        _set_command_source=lambda mode, reason: requests.append((mode, reason)) or True,
    )

    old_goal = goal_status(1, GoalStatus.STATUS_EXECUTING)
    FrontierExplorer._on_nav2_goal_status(gate, status_message(old_goal))
    assert requests == []

    new_goal = goal_status(2, GoalStatus.STATUS_EXECUTING)
    FrontierExplorer._on_nav2_goal_status(gate, status_message(old_goal, new_goal))
    assert gate._nav2_goal_active is True
    assert [mode for mode, _reason in requests] == ["NAV2"]

    old_goal.status = GoalStatus.STATUS_CANCELED
    FrontierExplorer._on_nav2_goal_status(gate, status_message(old_goal, new_goal))
    assert gate._nav2_goal_active is True
    assert [mode for mode, _reason in requests] == ["NAV2"]

    new_goal.status = GoalStatus.STATUS_ABORTED
    FrontierExplorer._on_nav2_goal_status(gate, status_message(old_goal, new_goal))
    assert gate._nav2_goal_active is False
