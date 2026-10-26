"""Test navigation lifecycle without ROS hardware or nvblox message installation."""
import ast
import time
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

SCRIPT = Path(__file__).parents[1] / 'scripts' / 'frontier_explorer.py'
sys.path.insert(0, str(SCRIPT.parent))
from frontier_search import short_path_goal
tree = ast.parse(SCRIPT.read_text())
node = next(n for n in tree.body if isinstance(n, ast.ClassDef))
namespace = {'math': math, 'time': time, 'short_path_goal': short_path_goal, 'Node': object, 'String': lambda **kwargs: NS(**kwargs),
             'GoalStatus': NS(STATUS_SUCCEEDED=4, STATUS_ABORTED=6),
             'NavigateToPose': NS(Goal=lambda: NS()),
             'Spin': NS(Goal=lambda: NS()),
             'TransformException': RuntimeError,
             'BackUp': NS(Goal=lambda: NS(target=NS(x=0.0, y=0.0, z=0.0))),
             'Duration': lambda **kwargs: NS(**kwargs),
             'PoseStamped': lambda: NS(header=NS(), pose=NS(position=NS(), orientation=NS()))}
exec(compile(ast.Module(body=[node], type_ignores=[]), str(SCRIPT), 'exec'), namespace)
Explorer = namespace['FrontierExplorer']


def explorer(dry_run=False):
    obj = Explorer.__new__(Explorer)
    obj.dry_run = dry_run
    obj.stopped = False
    obj.paused = False
    obj.pause_stop_confirmed = False
    obj._pending_actions = set()
    obj._pending_selectors = set()
    obj.path_pending = False
    obj.current_goal_xy = None
    obj.minimum_clearance = .45
    obj.goal_pending = True
    obj.goal_active = False
    obj.goal_handle = None
    obj._nav2_goal_active = False
    obj.goal_number = 1
    obj.returning_to_start = False
    obj.return_recovery_attempts = 0
    obj.navigation_failures = 0
    obj.successful_goal_count = 0
    obj.retry_after = 0.0
    obj.backup_distance = 0.20
    obj.backup_speed = 0.05
    obj.backup_time_allowance = 6.0
    obj.retry_return_after_backup = False
    obj.return_retry_pending = False
    obj.stop_after_backup = False
    obj.backup_handle = None
    obj.backup_attempt = 0
    obj.backup_client = Mock()
    obj.backup_client.server_is_ready.return_value = True
    obj.backup_future = Mock()
    obj.backup_client.send_goal_async.return_value = obj.backup_future
    obj._on_backup_goal_response = Mock()
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


def test_stale_map_during_active_goal_cancels_and_pauses():
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
    obj.successful_goal_count = 2
    obj._complete()
    assert obj.stopped
    assert obj._set_command_source.call_args.args[0] == 'STOP'
    assert obj.status_pub.publish.call_args.args[0].data.startswith('COMPLETE: 2 exploration goals reached')


def test_return_goal_uses_original_start_pose_without_counting_as_exploration():
    obj = explorer()
    obj.start_pose = (2.0, -1.0, math.pi / 2)
    obj._robot_pose = Mock(return_value=(1.8, -1.0))
    obj.map_frame = 'odom'
    obj.return_to_start = True
    obj.no_frontier_reported = True
    obj.get_clock = Mock(return_value=NS(now=Mock(return_value=NS(to_msg=lambda: 'stamp'))))
    obj.action_client = Mock()
    obj.action_client.server_is_ready.return_value = True
    future = Mock()
    obj.action_client.send_goal_async.return_value = future
    obj._on_goal_response = Mock()

    obj._return_to_start()

    goal = obj.action_client.send_goal_async.call_args.args[0]
    assert goal.pose.header.frame_id == 'odom'
    assert goal.pose.header.stamp == 'stamp'
    assert (goal.pose.pose.position.x, goal.pose.pose.position.y) == (2.0, -1.0)
    assert abs(goal.pose.pose.orientation.z - math.sin(-math.pi / 4)) < 1e-9
    assert obj.returning_to_start and obj.goal_pending
    assert obj.successful_goal_count == 0
    future.add_done_callback.assert_called_once()
    future.add_done_callback.call_args.args[0](future)
    obj._on_goal_response.assert_called_once_with(future)


def test_successful_return_finishes_only_after_reaching_start():
    obj = explorer()
    obj.returning_to_start = True
    obj.successful_goal_count = 4
    obj.visited_positions = [(0., 0.), (1., 0.)]

    obj._on_goal_result(NS(result=lambda: NS(status=4)))

    assert obj.stopped
    assert obj.successful_goal_count == 4
    assert 'returned to start position' in obj.status_pub.publish.call_args.args[0].data
    assert obj._set_command_source.call_args.args[0] == 'STOP'


def test_failed_return_requests_one_checked_backup_before_retry():
    obj = explorer()
    obj.returning_to_start = True
    obj.navigation_failures = 0
    obj._on_goal_result(NS(result=lambda: NS(status=6)))

    assert not obj.stopped
    assert obj.navigation_failures == 0
    assert obj.retry_return_after_backup
    obj.backup_client.send_goal_async.assert_called_once()


def test_aborted_frontier_requests_checked_backup_before_replanning():
    obj = explorer()
    obj.map_frame = 'odom'
    obj._robot_pose = Mock(return_value=(0.4, -0.2))
    obj.visited_positions = [(0., 0.)]
    obj.failed_positions = []
    obj.current_goal_xy = (0.6, -0.4)

    obj._on_goal_result(NS(result=lambda: NS(status=6)))

    backup_goal = obj.backup_client.send_goal_async.call_args.args[0]
    assert backup_goal.target.x == 0.20
    assert backup_goal.target.y == 0.0
    assert backup_goal.speed == 0.05
    assert backup_goal.time_allowance.sec == 6
    assert obj.visited_positions[-1] == (0.4, -0.2)
    assert obj.failed_positions == [(0.6, -0.4)]
    assert not obj.stopped
    obj.backup_future.add_done_callback.assert_called_once()
    obj.backup_future.add_done_callback.call_args.args[0](obj.backup_future)
    obj._on_backup_goal_response.assert_called_once_with(obj.backup_future)


def test_backup_success_schedules_frontier_replan_after_costmap_refresh():
    obj = explorer()
    obj.goal_active = True
    obj._on_backup_goal_result(NS(result=lambda: NS(status=4)))

    assert not obj.goal_active
    assert obj.retry_after > time.monotonic()
    assert 'REPLANNING' in obj.status_pub.publish.call_args.args[0].data
    assert obj._set_command_source.call_args.args[0] == 'STOP'


def test_backup_failure_retries_with_shorter_costmap_checked_distance():
    obj = explorer()
    obj.goal_active = True
    obj._on_backup_goal_result(NS(result=lambda: NS(status=6)))

    backup_goal = obj.backup_client.send_goal_async.call_args.args[0]
    assert not obj.stopped
    assert obj.backup_attempt == 1
    assert backup_goal.target.x == 0.10
    assert obj.goal_pending


def test_third_navigation_failure_backs_away_then_stops_for_inspection():
    obj = explorer()
    obj.navigation_failures = 2
    obj.map_frame = 'odom'
    obj._robot_pose = Mock(return_value=(0.4, -0.2))
    obj.visited_positions = [(0., 0.)]
    obj.failed_positions = []
    obj.current_goal_xy = (0.6, -0.4)

    obj._on_goal_result(NS(result=lambda: NS(status=6)))
    obj._on_backup_goal_result(NS(result=lambda: NS(status=4)))

    assert obj.stopped
    assert 'Three navigation failures' in obj.status_pub.publish.call_args.args[0].data


def test_blocked_backup_after_final_failure_tries_an_alternate_frontier():
    obj = explorer()
    obj.backup_attempt = 2
    obj.goal_active = True
    obj._on_backup_goal_result(NS(result=lambda: NS(status=6)))

    assert not obj.stopped
    assert obj.retry_after > time.monotonic()
    assert 'REPLANNING: reverse path blocked' in obj.status_pub.publish.call_args.args[0].data


def test_third_navigation_failure_stops_if_all_backup_distances_are_blocked():
    obj = explorer()
    obj.backup_attempt = 2
    obj.stop_after_backup = True
    obj.goal_active = True
    obj._on_backup_goal_result(NS(result=lambda: NS(status=6)))

    assert obj.stopped
    assert 'Three navigation failures' in obj.status_pub.publish.call_args.args[0].data


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
    obj.backup_client.send_goal_async.assert_called_once()


def test_abort_after_operator_stop_never_restarts_exploration():
    obj = explorer()
    obj.stopped = True
    obj._on_goal_result(NS(result=lambda: NS(status=6)))
    assert obj.stopped


def test_arrival_uses_in_place_spin_instead_of_another_translation_goal():
    obj = explorer()
    obj.visited_positions = []
    obj.align_after_arrival = True
    obj.map_frame = 'odom'
    obj.robot_yaw = 0.
    obj.current_goal_yaw = .4
    obj.current_goal_xy = (1., 0.)
    obj._robot_pose = Mock(return_value=(1., 0.))
    obj.orientation_client = Mock()
    obj.orientation_client.server_is_ready.return_value = True
    obj._on_goal_result(NS(result=lambda: NS(status=4)))
    assert obj.goal_pending
    assert obj.successful_goal_count == 0
    assert obj.orientation_client.send_goal_async.call_args.args[0].target_yaw == .4
    handle = Mock(accepted=True)
    obj._on_orientation_response(NS(result=lambda: handle))
    assert obj._set_command_source.call_args.args[0] == 'NAV2'
    obj._on_orientation_result(NS(result=lambda: NS(status=4)))
    assert obj.successful_goal_count == 1
    assert obj.visited_positions == [(1., 0.)]
    assert obj._set_command_source.call_args.args[0] == 'STOP'


def test_late_orientation_acceptance_after_stop_is_cancelled():
    obj = explorer()
    obj.stopped = True
    handle = Mock(accepted=True)
    obj._on_orientation_response(NS(result=lambda: handle))
    handle.cancel_goal_async.assert_called_once()
    obj._set_command_source.assert_not_called()


def test_intermediate_return_step_does_not_claim_return_complete():
    obj = explorer()
    obj.returning_to_start = True
    obj.return_step_is_final = False
    obj._complete = Mock()
    obj._finish_arrival()
    assert obj.return_retry_pending
    obj._complete.assert_not_called()
    obj.return_step_is_final = True
    obj._finish_arrival()
    obj._complete.assert_called_once_with(returned_to_start=True)


def test_short_waypoints_may_leave_the_exploration_rectangle():
    def pose(x, y):
        return NS(pose=NS(position=NS(x=x, y=y), orientation=NS(x=0., y=0., z=0., w=1.)))
    for returning in (False, True):
        obj = explorer()
        obj.returning_to_start = returning
        obj.start_pose = (0., 0., 0.)
        obj.map_frame = 'odom'
        obj.map_msg = NS()
        obj.maximum_step_distance = .6
        obj.path_target = (1., -1., 0., .45, 3)
        obj._robot_pose = Mock(return_value=(.4, -.1))
        obj._send_navigation_goal = Mock()
        obj._publish_markers = Mock()
        obj._short_path_failed = Mock()
        path = NS(header=NS(frame_id='odom'), poses=[pose(.4, -.1), pose(-.4, -.1), pose(0., 0.)])
        obj._on_short_path_result(NS(result=lambda: NS(status=4, result=NS(path=path))))
        obj._short_path_failed.assert_not_called()
        obj._send_navigation_goal.assert_called_once()
        assert obj._send_navigation_goal.call_args.args[0] < 0.


def test_home_translation_heading_is_separate_from_saved_arrival_heading():
    obj = explorer()
    obj.start_pose = (0., 0., .1)
    obj.return_to_start = True
    obj.map_frame = 'odom'
    obj.maximum_step_distance = .6
    obj.align_after_arrival = True
    obj._robot_pose = Mock(return_value=(.4, -.1))
    obj.get_clock = Mock(return_value=NS(now=Mock(return_value=NS(to_msg=lambda: 'stamp'))))
    obj.action_client = Mock()
    obj.action_client.server_is_ready.return_value = True
    obj._return_to_start()
    goal = obj.action_client.send_goal_async.call_args.args[0]
    expected = math.atan2(.1, -.4)
    assert goal.pose.pose.orientation.z == math.sin(expected/2)
    assert math.isclose(obj.current_goal_yaw, .1-math.pi)


def test_arrival_waits_for_stop_ack_before_requesting_detection():
    namespace['Trigger'] = NS(Request=lambda: NS())
    obj = explorer()
    obj.checkpoint_detection = True
    obj.detection_client = Mock()
    obj.detection_client.service_is_ready.return_value = True
    obj.detection_future = None
    obj._finish_arrival()
    assert obj.detection_waiting
    obj.detection_client.call_async.assert_not_called()
    assert obj.successful_goal_count == 0
    # An older STOP acknowledgement must not trigger this checkpoint.
    obj._on_command_source_result(NS(result=lambda: NS(results=[NS(successful=True)])),
                                  'STOP', 'frontier goal finished')
    obj.detection_client.call_async.assert_not_called()
    obj._on_command_source_result(NS(result=lambda: NS(results=[NS(successful=True)])),
                                  'STOP', 'stationary person detection')
    obj.detection_client.call_async.assert_called_once()
    assert not obj.detection_waiting
    obj.visited_positions = []
    obj.detection_future.done.return_value = True
    obj.detection_future.result.return_value = NS(success=True, message='3 frames')
    obj._poll_detection()
    assert obj.successful_goal_count == 1
    assert obj.detection_future is None


def test_detection_failure_prevents_next_navigation():
    obj = explorer()
    obj.detection_future = NS(done=lambda: True, result=lambda: NS(success=False, message='timeout'))
    obj._poll_detection()
    assert obj.stopped
    assert obj.successful_goal_count == 0


def test_return_transit_does_not_run_detector():
    obj = explorer()
    obj.checkpoint_detection = True
    obj.returning_to_start = True
    obj.return_step_is_final = False
    obj.detection_client = Mock()
    obj._finish_arrival()
    assert obj.return_retry_pending
    obj.detection_client.call_async.assert_not_called()


def test_paused_return_drains_terminal_result_before_resume_and_retains_home():
    from concurrent.futures import Future
    obj = explorer()
    obj.returning_to_start = True
    obj.start_pose = (0., 0., .2)
    obj.area_yaw = .2
    obj.successful_goal_count = 7
    obj.visited_positions = [(0., 0.), (1., -1.)]
    obj.goal_handle = Mock()
    old_result = Future()
    obj._watch_action(old_result, obj._on_goal_result)
    obj._pause_for_video('depth unavailable for 10s')
    assert obj.stopped and obj.paused
    obj.goal_handle.cancel_goal_async.assert_called_once()
    assert not obj._resume_after_video()
    obj.pause_stop_confirmed = True
    assert not obj._resume_after_video()
    old_result.set_result(NS(status=5))
    assert obj._resume_after_video()
    assert obj.return_retry_pending
    assert obj.start_pose == (0., 0., .2) and obj.area_yaw == .2
    assert obj.visited_positions == [(0., 0.), (1., -1.)]
    assert obj.successful_goal_count == 7
    assert obj.goal_handle is None and not obj.goal_active


def test_late_goal_acceptance_during_pause_is_cancelled_and_drained():
    from concurrent.futures import Future
    obj = explorer()
    obj.returning_to_start = True
    accepted = Future()
    result = Future()
    callback = Mock()
    obj._watch_action(accepted, callback, response=True)
    obj._pause_for_video('depth gap')
    obj.pause_stop_confirmed = True
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = result
    accepted.set_result(handle)
    handle.cancel_goal_async.assert_called_once()
    callback.assert_not_called()
    assert not obj._resume_after_video()
    result.set_result(NS(status=5))
    assert obj._resume_after_video()


def test_operator_stop_clears_pause_and_never_resumes():
    obj = explorer()
    obj.returning_to_start = True
    obj._pause_for_video('gap')
    obj._on_source_request(NS(data='STOP'))
    assert obj.stopped and not obj.paused
    obj.pause_stop_confirmed = True
    assert not obj._resume_after_video()


def test_selector_refusal_during_pause_makes_stop_permanent():
    obj = explorer()
    obj.returning_to_start = True
    obj._pause_for_video('gap')
    obj._on_command_source_result(NS(result=lambda: NS(results=[NS(successful=False, reason='refused')])),
                                 'STOP', 'video outage pause')
    assert obj.stopped and not obj.paused
    assert not obj._resume_after_video()


def test_pause_during_detection_waits_for_service_to_finish_then_retries_target():
    from concurrent.futures import Future
    obj = explorer()
    obj.current_goal_xy = (1., -.5)
    obj.current_goal_yaw = .3
    obj.detection_future = Future()
    obj._pause_for_video('gap')
    obj.pause_stop_confirmed = True
    assert not obj._resume_after_video()
    obj.detection_future.set_result(NS(success=True))
    assert obj._resume_after_video()
    assert obj.resume_target_pending == (1., -.5, .3, .45, 0)
    assert obj.detection_future is None
    assert obj.successful_goal_count == 0


def test_return_heading_is_opposite_start_without_rotating_area():
    obj = explorer()
    for yaw in (0., .2, -2.8, math.pi):
        obj.start_pose = (1., 2., yaw)
        expected = yaw + math.pi
        actual = obj._return_heading()
        assert math.isclose(math.sin(actual), math.sin(expected), abs_tol=1e-9)
        assert math.isclose(math.cos(actual), math.cos(expected), abs_tol=1e-9)
        assert obj.start_pose == (1., 2., yaw)
    obj.return_heading_offset = 0.
    assert math.isclose(obj._return_heading(), math.pi)


def test_final_short_return_step_uses_opposite_arrival_heading():
    obj = explorer()
    obj.returning_to_start = True
    obj.start_pose = (0., 0., .2)
    obj.map_frame = 'odom'
    obj.map_msg = NS()
    obj.maximum_step_distance = .6
    obj.path_target = (0., 0., .2, .45, 0)
    obj._robot_pose = Mock(return_value=(.3, 0.))
    obj._send_navigation_goal = Mock()
    obj._publish_markers = Mock()
    obj._short_path_failed = Mock()
    def pose(x, y):
        return NS(pose=NS(position=NS(x=x, y=y), orientation=NS(x=0., y=0., z=0., w=1.)))
    path = NS(header=NS(frame_id='odom'), poses=[pose(.3, 0.), pose(0., 0.)])
    obj._on_short_path_result(NS(result=lambda: NS(status=4, result=NS(path=path))))
    obj._short_path_failed.assert_not_called()
    assert obj.return_step_is_final
    actual = obj._send_navigation_goal.call_args.kwargs['arrival_yaw']
    assert math.isclose(actual, .2-math.pi)
