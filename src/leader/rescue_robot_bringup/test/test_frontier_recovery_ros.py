"""Cancel/drain/resume against real ROS action and parameter services, no robot."""
import math
from pathlib import Path
import sys
import threading
import time

import pytest
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('nvblox_msgs.msg')
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from frontier_explorer import FrontierExplorer


def test_video_pause_cancels_real_action_and_resumes_only_after_stop_ack():
    rclpy.init(args=[])
    explorer = FrontierExplorer()
    explorer.timer.cancel()
    explorer.dry_run = False
    explorer.returning_to_start = True
    explorer.start_pose = (0., 0., .2)
    source = rclpy.create_node('recovery_fake_nav2')
    cancelled = threading.Event()
    def execute(handle):
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            if handle.is_cancel_requested:
                handle.canceled()
                cancelled.set()
                return NavigateToPose.Result()
            time.sleep(.01)
        handle.abort()
        return NavigateToPose.Result()
    def parameters(request, response):
        response.results = [SetParametersResult(successful=True) for _ in request.parameters]
        return response
    group = ReentrantCallbackGroup()
    server = ActionServer(source, NavigateToPose, '/navigate_to_pose', execute_callback=execute,
                          cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
    service = source.create_service(SetParameters, '/leader/command_selector/set_parameters',
                                    parameters, callback_group=group)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(explorer)
    executor.add_node(source)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    def wait_for(predicate):
        deadline = time.monotonic()+6
        while time.monotonic()<deadline:
            if predicate():
                return
            time.sleep(.01)
        assert predicate()
    try:
        assert explorer.action_client.wait_for_server(timeout_sec=5)
        assert explorer.selector_parameter_client.wait_for_service(timeout_sec=5)
        explorer.goal_pending = True
        explorer._watch_action(explorer.action_client.send_goal_async(NavigateToPose.Goal()),
                               explorer._on_goal_response, response=True)
        wait_for(lambda: explorer.goal_active)
        explorer._pause_for_video('injected video gap')
        assert explorer.paused and explorer.stopped
        wait_for(lambda: cancelled.is_set() and explorer._pause_actions_settled())
        assert explorer._resume_after_video()
        assert explorer.return_retry_pending
        assert explorer.start_pose == (0., 0., .2)
        assert math.isclose(explorer._return_heading(), .2-math.pi)
    finally:
        executor.shutdown()
        thread.join()
        server.destroy()
        source.destroy_service(service)
        explorer.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
