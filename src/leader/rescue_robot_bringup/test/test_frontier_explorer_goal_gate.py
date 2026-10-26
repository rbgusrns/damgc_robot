"""Test navigation lifecycle without ROS hardware or nvblox message installation."""
import ast
import time
import math
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

SCRIPT = Path(__file__).parents[1] / 'scripts' / 'frontier_explorer.py'
tree = ast.parse(SCRIPT.read_text())
node = next(n for n in tree.body if isinstance(n, ast.ClassDef))
namespace = {'math': math, 'time': time, 'Node': object, 'String': lambda **kwargs: NS(**kwargs),
             'GoalStatus': NS(STATUS_SUCCEEDED=4, STATUS_ABORTED=6)}
exec(compile(ast.Module(body=[node], type_ignores=[]), str(SCRIPT), 'exec'), namespace)
Explorer = namespace['FrontierExplorer']


def explorer(dry_run=False):
    obj = Explorer.__new__(Explorer)
    obj.dry_run = dry_run
    obj.stopped = False
    obj.goal_pending = True
    obj.goal_active = False
    obj.goal_handle = None
    obj._nav2_goal_active = False
    obj.goal_number = 1
    obj._set_command_source = Mock(return_value=True)
    obj.status_pub = Mock()
    obj.get_logger = Mock(return_value=Mock())
    return obj


def test_stop_while_goal_pending_cancels_late_acceptance():
    obj = explorer()
    obj._stop('stop while pending')
    handle = Mock(accepted=True)
    obj._on_goal_response(NS(result=lambda: handle))
    handle.cancel_goal_async.assert_called_once()
    assert not obj.goal_active
    assert obj._set_command_source.call_args.args[0] == 'STOP'


def test_owned_goal_acquires_nav2_and_result_releases_source():
    obj = explorer()
    handle = Mock(accepted=True)
    obj._on_goal_response(NS(result=lambda: handle))
    assert obj.goal_active
    assert obj._set_command_source.call_args.args[0] == 'NAV2'
    obj._on_goal_result(NS(result=lambda: NS(status=4)))
    assert not obj.goal_active
    assert obj._set_command_source.call_args.args[0] == 'STOP'


def test_selector_unavailable_cancels_owned_goal():
    obj = explorer()
    obj._set_command_source.return_value = False
    handle = Mock(accepted=True)
    obj._on_goal_response(NS(result=lambda: handle))
    assert obj.stopped
    handle.cancel_goal_async.assert_called_once()


def test_preview_stop_never_changes_source_or_cancels_navigation():
    obj = explorer(dry_run=True)
    obj.goal_handle = Mock()
    obj._stop('preview ended')
    obj._set_command_source.assert_not_called()
    obj.goal_handle.cancel_goal_async.assert_not_called()


def test_manual_takeover_stops_exploration():
    obj = explorer()
    obj.goal_handle = Mock()
    obj._on_source_request(NS(data='TELEOP'))
    assert obj.stopped
    obj.goal_handle.cancel_goal_async.assert_called_once()


def test_rejected_goal_stops_exploration():
    obj = explorer()
    obj._on_goal_response(NS(result=lambda: NS(accepted=False)))
    assert obj.stopped
    assert not obj.goal_active


def test_preview_updates_multiple_times_without_sending_goals():
    obj = explorer(dry_run=True)
    obj.goal_pending = False
    obj.map_frame = 'odom'
    obj.map_msg = NS(header=NS(frame_id='odom'))
    obj.map_received_at = time.monotonic()
    obj.map_stale_sec = 2.
    obj.area_center = (0., 0.)
    obj.visited_positions = []
    obj._robot_pose = Mock(return_value=(0., 0.))
    obj._find_frontier_clusters = Mock(return_value=[[(1., 0., .5, 1.1, 0.)]])
    obj._choose_goal = Mock(side_effect=[((1., 0., 0., .5, 5), 1),
                                        ((0., 1., 0., .5, 5), 1)])
    obj._publish_markers = Mock()
    obj.last_dry_goal = None
    obj.action_client = Mock()
    obj._tick()
    obj._tick()
    assert obj.last_dry_goal == (0., 1.)
    assert not obj.stopped
    obj.action_client.send_goal_async.assert_not_called()
    obj._set_command_source.assert_not_called()


def test_stale_map_during_active_goal_cancels_and_stops():
    obj = explorer()
    obj.map_frame = 'odom'
    obj.map_msg = NS(header=NS(frame_id='odom'))
    obj.map_received_at = time.monotonic() - 3.
    obj.map_stale_sec = 2.
    obj.goal_handle = Mock()
    obj._tick()
    assert obj.stopped
    obj.goal_handle.cancel_goal_async.assert_called_once()


def test_forward_frontier_wins_over_closer_camera_side_edge():
    obj = explorer()
    obj.robot_yaw = 0.
    side = [(0.375, .325, .46, .3, .4)] * 5
    forward = [(1.5, .05, .5, 1.6, .05)] * 8
    goal, _ = obj._choose_goal(None, 0., 0., [side, forward])
    assert goal[:2] == (1.5, .05)
    assert abs(goal[2]) < .05


def test_forward_preference_follows_robot_heading():
    obj = explorer()
    obj.robot_yaw = math.pi/2
    goal, _ = obj._choose_goal(None, 0., 0.,
                              [[(.5, 0., .5, .6, 0.)], [(0., 1., .5, 0., 1.1)]])
    assert goal[:2] == (0., 1.)


def test_arrival_heading_follows_travel_instead_of_unknown_edge():
    obj = explorer()
    obj.robot_yaw = 0.
    goal, _ = obj._choose_goal(None, 0., 0., [[(.8, 0., .5, .8, .1)]])
    assert goal[2] == 0.


def test_preferred_distance_avoids_tiny_forward_steps():
    obj = explorer()
    obj.robot_yaw = 0.
    obj.preferred_goal_distance = 1.5
    goal, _ = obj._choose_goal(None, 0., 0., [[(.65, 0., .5, .7, 0.),
                                               (1.5, 0., .5, 1.6, 0.)]])
    assert goal[:2] == (1.5, 0.)


def test_completion_reports_coverage_and_stops_source():
    obj = explorer()
    obj.visited_positions = [(0., 0.), (1.5, 0.), (-1., 0.)]
    obj._complete()
    assert obj.stopped
    assert obj._set_command_source.call_args.args[0] == 'STOP'
    assert obj.status_pub.publish.call_args.args[0].data.startswith('COMPLETE: 2 goals reached')


def test_completed_frontiers_do_not_send_robot_back_to_same_boundary():
    obj = explorer()
    obj.visited_positions = [(0., 0.), (1.5, 0.)]
    obj.minimum_cluster_cells = 1
    old = [(1.5, .1, .6, 1.5, .2)]
    new = [(2.5, .1, .6, 2.5, .2)]
    assert obj._unvisited_frontiers([old, new]) == [new]


def test_failed_goal_is_excluded_without_lowering_clearance():
    obj = explorer()
    obj.failed_positions = [(1.5, 0.)]
    obj.robot_yaw = 0.
    goal, _ = obj._choose_goal(None, 0., 0., [[(1.5, .1, .6, 1.5, .2),
                                            (2.5, .1, .6, 2.5, .2)]])
    assert goal[:2] == (2.5, .1)


def test_abort_records_actual_observation_without_counting_goal_success():
    obj = explorer()
    obj.map_frame = 'odom'
    obj._robot_pose = Mock(return_value=(1.4, 0.))
    obj.visited_positions = [(0., 0.)]
    obj.failed_positions = []
    obj.current_goal_xy = (1.5, 0.)
    obj.successful_goal_count = 0
    obj._on_goal_result(NS(result=lambda: NS(status=6)))
    assert obj.successful_goal_count == 0
    assert obj.failed_positions == [(1.5, 0.)]
    assert not obj.stopped
    assert obj.retry_after > time.monotonic()


def test_abort_after_operator_stop_never_restarts_exploration():
    obj = explorer()
    obj.stopped = True
    obj._on_goal_result(NS(result=lambda: NS(status=6)))
    assert obj.stopped
