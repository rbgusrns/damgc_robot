#!/usr/bin/env python3
"""Greedy frontier exploration bounded around the starting pose.

Frontiers are detected directly in nvblox's DistanceMapSlice so unobserved
cells remain distinguishable from free cells in Nav2's merged costmap.
"""

from frontier_search import frontier_clusters, short_path_goal
import math
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav2_msgs.action import BackUp, ComputePathToPose, NavigateToPose, Spin
from nvblox_msgs.msg import DistanceMapSlice
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray



class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        self.declare_parameter("map_topic", "/nvblox_node/static_map_slice")
        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("area_radius_m", 0.0)
        self.declare_parameter("minimum_clearance_m", 0.60)
        self.declare_parameter("connection_clearance_m", 0.20)
        self.declare_parameter("minimum_goal_distance_m", 0.60)
        self.declare_parameter("preferred_goal_distance_m", 1.5)
        self.declare_parameter("minimum_cluster_cells", 5)
        self.declare_parameter("minimum_unknown_area_m2", 0.10)
        self.declare_parameter("no_frontier_confirm_sec", 5.0)
        self.declare_parameter("map_stale_sec", 10.0)
        self.declare_parameter("robot_transform_stale_sec", 3.0)
        self.declare_parameter("maximum_step_distance_m", 0.60)
        self.declare_parameter("align_after_arrival", True)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("area_width_m", 3.0)
        self.declare_parameter("area_height_m", 3.0)
        self.declare_parameter("area_offset_x_m", 1.5)
        self.declare_parameter("area_offset_y_m", -1.5)
        self.declare_parameter("area_align_with_start_heading", True)
        self.declare_parameter("goal_timeout_sec", 90.0)
        self.declare_parameter("failed_goal_exclusion_m", 1.0)
        self.declare_parameter("return_to_start", True)
        self.declare_parameter("backup_action_name", "/backup")
        self.declare_parameter("backup_distance_m", 0.20)
        self.declare_parameter("backup_speed_mps", 0.05)
        self.declare_parameter("backup_time_allowance_sec", 6.0)
        self.declare_parameter("completed_goal_count", -1)
        self.declare_parameter("visited_positions", [],
                               descriptor=ParameterDescriptor(dynamic_typing=True))

        self.area_radius = float(self.get_parameter("area_radius_m").value)
        self.minimum_clearance = float(self.get_parameter("minimum_clearance_m").value)
        self.connection_clearance = float(self.get_parameter("connection_clearance_m").value)
        self.minimum_goal_distance = float(self.get_parameter("minimum_goal_distance_m").value)
        self.preferred_goal_distance = float(self.get_parameter("preferred_goal_distance_m").value)
        self.minimum_cluster_cells = int(self.get_parameter("minimum_cluster_cells").value)
        self.minimum_unknown_area = float(self.get_parameter("minimum_unknown_area_m2").value)
        self.no_frontier_confirm_sec = float(
            self.get_parameter("no_frontier_confirm_sec").value
        )
        self.map_stale_sec = float(self.get_parameter("map_stale_sec").value)
        self.robot_transform_stale_sec = float(self.get_parameter("robot_transform_stale_sec").value)
        self.maximum_step_distance = float(self.get_parameter("maximum_step_distance_m").value)
        self.align_after_arrival = bool(self.get_parameter("align_after_arrival").value)
        self.step_behavior_tree = str(Path(get_package_share_directory("rescue_robot_bringup")) /
                                      "config/navigate_to_pose_steps.xml")
        self.dry_run = bool(self.get_parameter("dry_run").value)


        self.area_width = float(self.get_parameter("area_width_m").value)
        self.area_height = float(self.get_parameter("area_height_m").value)
        self.area_offset_x = float(self.get_parameter("area_offset_x_m").value)
        self.area_offset_y = float(self.get_parameter("area_offset_y_m").value)
        self.area_align_with_start_heading = bool(
            self.get_parameter("area_align_with_start_heading").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
        self.failed_goal_exclusion = float(
            self.get_parameter("failed_goal_exclusion_m").value)
        self.return_to_start = bool(self.get_parameter("return_to_start").value)
        self.backup_distance = float(self.get_parameter("backup_distance_m").value)
        self.backup_speed = float(self.get_parameter("backup_speed_mps").value)
        self.backup_time_allowance = float(
            self.get_parameter("backup_time_allowance_sec").value)
        positive = (self.minimum_clearance, self.minimum_goal_distance, self.preferred_goal_distance,
                    self.map_stale_sec, self.robot_transform_stale_sec, self.maximum_step_distance,
                    self.no_frontier_confirm_sec, self.goal_timeout,
                    self.failed_goal_exclusion, self.backup_distance, self.backup_speed,
                    self.backup_time_allowance)
        if (not math.isfinite(self.connection_clearance) or self.connection_clearance <= 0
                or self.connection_clearance > self.minimum_clearance):
            raise ValueError("Connection clearance must be positive and <= goal clearance")
        if not math.isfinite(self.minimum_unknown_area) or self.minimum_unknown_area < 0:
            raise ValueError("Minimum unknown area must be finite and nonnegative")
        if any(not math.isfinite(v) or v <= 0 for v in positive):
            raise ValueError("Distances and timeouts must be finite and positive")
        if (not math.isfinite(self.area_radius) or self.area_radius < 0
                or not all(math.isfinite(v) for v in (self.area_width, self.area_height,
                                              self.area_offset_x, self.area_offset_y))
                or self.area_width < 0 or self.area_height < 0
                or bool(self.area_width) != bool(self.area_height)
                or self.minimum_cluster_cells < 1):
            raise ValueError("Set both positive rectangle dimensions, or both zero")
        self.goal_handle = None
        restored = list(self.get_parameter("visited_positions").value or [])
        if len(restored) % 2 or any(not math.isfinite(float(v)) for v in restored):
            raise ValueError("visited_positions must contain finite x,y pairs from this map session")
        self.visited_positions = list(zip(restored[::2], restored[1::2]))
        self.successful_goal_count = int(self.get_parameter("completed_goal_count").value)
        if self.successful_goal_count < 0:
            self.successful_goal_count = max(0, len(self.visited_positions)-1)
        self.failed_positions = []
        self.navigation_failures = 0
        self.return_recovery_attempts = 0
        self.stop_after_backup = False
        self.retry_return_after_backup = False
        self.return_retry_pending = False
        self.backup_handle = None
        self.backup_attempt = 0
        self.retry_after = 0.0
        self.current_goal_xy = None
        self.current_goal_yaw = 0.0
        self.return_step_is_final = True
        self.path_pending = False
        self.path_handle = None
        self.orientation_handle = None
        self.goal_started_at = None
        self.map_frame = None
        self.status_pub = self.create_publisher(String, "/frontier_explorer/status", 10)
        self.create_service(Trigger, "/frontier_explorer/stop", self._stop_request)
        self.create_subscription(String, "/leader/command_selector/request",
                                 self._on_source_request, 10)
        self.tf_buffer = Buffer()
        # Keep TF callbacks independent of image/graph-search work. A separate
        # listener node avoids accumulating old transforms on our executor.
        self.tf_listener_node = Node(
            "frontier_transform_listener", use_global_arguments=False,
            enable_rosout=False, start_parameter_services=False,
        )
        self.tf_listener = TransformListener(
            self.tf_buffer, self.tf_listener_node, spin_thread=True,
            qos=QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE))
        self.map_msg = None
        self.map_received_at = 0.0
        self.area_center = None
        self.area_yaw = 0.0
        self.start_pose = None
        self.returning_to_start = False
        self.goal_pending = False
        self.goal_active = False
        self.stopped = False
        self.no_frontier_since = None
        self.goal_number = max(0, len(self.visited_positions)-1)
        self.last_dry_goal = None
        self.no_frontier_reported = False
        self._nav2_goal_active = False
        self._selector_enable_requested = False
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/frontier_explorer/markers", marker_qos
        )

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.map_sub = self.create_subscription(
            DistanceMapSlice,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            qos,
        )
        self.action_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter("action_name").value)
        )
        self.backup_client = ActionClient(
            self, BackUp, str(self.get_parameter("backup_action_name").value)
        )
        self.path_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.orientation_client = ActionClient(self, Spin, "/spin")
        self.selector_parameter_client = self.create_client(
            SetParameters, "/leader/command_selector/set_parameters"
        )
        self.timer = self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f"Frontier exploration ready: radius={self.area_radius:.2f} m, "
            f"clearance={self.minimum_clearance:.2f} m, dry_run={self.dry_run}"
        )

    def destroy_node(self):
        listener = getattr(self, "tf_listener", None)
        if listener is not None and hasattr(listener, "executor"):
            listener.executor.shutdown()
            listener.dedicated_listener_thread.join()
            listener.node.destroy_node()
        return super().destroy_node()

    def _on_map(self, msg):
        self.map_msg = msg
        self.map_received_at = time.monotonic()

    def _set_command_source(self, source, reason):
        if not self.selector_parameter_client.service_is_ready():
            self.get_logger().warning(
                f"Cannot set command selector to {source}: parameter service unavailable"
            )
            return False
        request = SetParameters.Request()
        request.parameters = [Parameter("source_mode", value=source).to_parameter_msg()]
        future = self.selector_parameter_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._on_command_source_result(completed, source, reason)
        )
        return True

    def _on_command_source_result(self, future, source, reason):
        try:
            results = future.result().results
        except Exception as exc:
            self.get_logger().error(
                f"Failed to set command selector to {source}: {exc}"
            )
            self._selector_enable_requested = False
            if source == "NAV2":
                self._stop("Could not acquire navigation command source")
            return
        if not results or not results[0].successful:
            detail = results[0].reason if results else "no parameter result"
            self.get_logger().error(
                f"Command selector refused {source}: {detail}"
            )
            self._selector_enable_requested = False
            if source == "NAV2":
                self._stop("Command selector refused navigation ownership")
            return
        self.get_logger().info(f"Command selector set to {source}: {reason}")
        if source == "NAV2" and not self._nav2_goal_active:
            self._selector_enable_requested = False
            self._set_command_source(
                "STOP", "goal ended before the selector update completed"
            )

    def _robot_pose(self, frame_id):
        transform = self.tf_buffer.lookup_transform(
            frame_id,
            str(self.get_parameter("base_frame").value),
            rclpy.time.Time(),
        )
        stamp = transform.header.stamp
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(stamp)).nanoseconds / 1e9
        if age > getattr(self, "robot_transform_stale_sec", self.map_stale_sec) or age < -0.5:
            raise TransformException(f"Robot transform is stale or clock is inconsistent (age={age:.3f}s)")
        p = transform.transform.translation
        q = transform.transform.rotation
        self.robot_yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                                    1 - 2*(q.y*q.y + q.z*q.z))
        return float(p.x), float(p.y)

    def _find_frontier_clusters(self, msg, robot_x, robot_y):
        self.search_stats = {}
        return frontier_clusters(
            msg, (robot_x, robot_y), self.area_center, self.area_radius,
            self.minimum_clearance, self.minimum_goal_distance, self.minimum_cluster_cells,
            self.area_width, self.area_height, self.area_offset_x, self.area_offset_y,
            connection_clearance=self.connection_clearance,
            minimum_unknown_area=self.minimum_unknown_area, stats=self.search_stats,
            area_yaw=self.area_yaw)

    def _unvisited_frontiers(self, clusters):
        minimum = getattr(self, "minimum_cluster_cells", 1)
        visits = getattr(self, "visited_positions", [])
        fresh = [[p for p in cluster if all(
            math.hypot(p[0]-x, p[1]-y) >= 0.60 for x, y in visits)]
            for cluster in clusters]
        return [cluster for cluster in fresh if len(cluster) >= minimum]

    def _choose_goal(self, msg, robot_x, robot_y, clusters=None):
        if clusters is None:
            clusters = self._find_frontier_clusters(msg, robot_x, robot_y)
        best = None
        best_rank = (True, math.inf)
        robot_yaw = getattr(self, "robot_yaw", 0.0)
        for cluster in clusters:
            for x, y, clearance, unknown_x, unknown_y in cluster:
                if any(math.hypot(x-fx, y-fy) < getattr(self, "failed_goal_exclusion", 1.0)
                       for fx, fy in getattr(self, "failed_positions", [])):
                    continue
                yaw = math.atan2(y - robot_y, x - robot_x)
                turn = abs(math.atan2(math.sin(yaw-robot_yaw), math.cos(yaw-robot_yaw)))
                travel = math.hypot(x-robot_x, y-robot_y)
                # Prefer observed, reachable frontiers in a 30-degree forward
                # cone before turning toward the camera's lateral blind edge.
                rank = (turn > math.pi/6,
                        0.35*abs(travel-getattr(self, "preferred_goal_distance", 1.5))
                        + 1.2*turn + 1.2/max(clearance, 0.01) - 0.12*math.sqrt(len(cluster)))
                if rank < best_rank:
                    best_rank = rank
                    best = (x, y, yaw, clearance, len(cluster))
        return best, len(clusters)

    def _publish_markers(self, msg, clusters, goal=None):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        frontiers = Marker()
        frontiers.header = msg.header
        frontiers.ns = "frontier_candidates"
        frontiers.id = 0
        frontiers.type = Marker.POINTS
        frontiers.action = Marker.ADD
        frontiers.pose.orientation.w = 1.0
        frontiers.scale.x = 0.07
        frontiers.scale.y = 0.07
        frontiers.color.r = 0.1
        frontiers.color.g = 0.9
        frontiers.color.b = 1.0
        frontiers.color.a = 1.0
        for cluster in clusters:
            for x, y, _clearance, _unknown_x, _unknown_y in cluster:
                point = Point()
                point.x, point.y, point.z = x, y, 0.10
                frontiers.points.append(point)
        markers.markers.append(frontiers)

        if self.area_center is not None:
            center = Marker()
            center.header = msg.header
            center.ns = "exploration_area"
            center.id = 0
            center.type = Marker.SPHERE
            center.action = Marker.ADD
            center.pose.position.x = self.area_center[0]
            center.pose.position.y = self.area_center[1]
            center.pose.position.z = 0.12
            center.pose.orientation.w = 1.0
            center.scale.x = center.scale.y = center.scale.z = 0.16
            center.color.r = 0.2
            center.color.g = 1.0
            center.color.b = 0.2
            center.color.a = 1.0
            markers.markers.append(center)

            boundary = Marker()
            boundary.header = msg.header
            boundary.ns = "exploration_area"
            boundary.id = 1
            boundary.type = Marker.LINE_STRIP
            boundary.action = Marker.ADD
            boundary.pose.orientation.w = 1.0
            boundary.scale.x = 0.025
            boundary.color.r = 0.2
            boundary.color.g = 1.0
            boundary.color.b = 0.2
            boundary.color.a = 0.8
            if self.area_width:
                w, h = self.area_width/2, self.area_height/2
                local_points = [(self.area_offset_x-w, self.area_offset_y-h),
                                (self.area_offset_x+w, self.area_offset_y-h),
                                (self.area_offset_x+w, self.area_offset_y+h),
                                (self.area_offset_x-w, self.area_offset_y+h),
                                (self.area_offset_x-w, self.area_offset_y-h)]
                cosine, sine = math.cos(self.area_yaw), math.sin(self.area_yaw)
                points = [(self.area_center[0] + cosine*x - sine*y,
                           self.area_center[1] + sine*x + cosine*y)
                          for x, y in local_points]
            elif self.area_radius:
                points = [(self.area_center[0] + self.area_radius*math.cos(i*math.pi/32),
                           self.area_center[1] + self.area_radius*math.sin(i*math.pi/32))
                          for i in range(65)]
            else:
                points = []
            for x, y in points:
                boundary.points.append(Point(x=x, y=y, z=0.03))
            markers.markers.append(boundary)

        if goal is not None:
            x, y, yaw, clearance, _cluster_size = goal
            target = Marker()
            target.header = msg.header
            target.ns = "selected_frontier"
            target.id = 1
            target.type = Marker.ARROW
            target.action = Marker.ADD
            target.pose.position.x = x
            target.pose.position.y = y
            target.pose.position.z = 0.18
            target.pose.orientation.z = math.sin(yaw * 0.5)
            target.pose.orientation.w = math.cos(yaw * 0.5)
            target.scale.x = 0.35
            target.scale.y = 0.08
            target.scale.z = 0.12
            target.color.r = 1.0
            target.color.g = 0.2
            target.color.b = 0.1
            target.color.a = 1.0
            target.text = f"goal; clearance {clearance:.2f} m"
            markers.markers.append(target)
        self.marker_pub.publish(markers)

    def _on_source_request(self, message):
        if not self.dry_run and not self.stopped and message.data.upper() != "NAV2":
            self._stop("Operator changed command source; exploration stopped")

    def _stop_request(self, request, response):
        self._stop("operator requested stop")
        response.success = True
        response.message = "Exploration stopped; restart the launch to explore again"
        return response

    def _stop(self, reason):
        self.stopped = True
        self._nav2_goal_active = False
        if not self.dry_run:
            self._set_command_source("STOP", reason)
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            if self.backup_handle is not None:
                self.backup_handle.cancel_goal_async()
            for handle in (getattr(self, "path_handle", None), getattr(self, "orientation_handle", None)):
                if handle is not None:
                    handle.cancel_goal_async()
        self.status_pub.publish(String(data="STOPPED: " + reason))
        self.get_logger().warning(reason)

    def _complete(self, returned_to_start=False):
        # Completion means reachable goal-space coverage at the configured
        # clearance/resolution, not proof about unseen rooms behind obstacles.
        self.stopped = True
        self._nav2_goal_active = False
        if not self.dry_run:
            self._set_command_source("STOP", "reachable exploration goals covered")
        count = getattr(self, "successful_goal_count", max(0, len(self.visited_positions)-1))
        return_text = "returned to start pose" if returned_to_start else "return to start disabled"
        text = (f"COMPLETE: {count} exploration goals reached; {return_text}; no eligible "
                "frontier or unvisited observation point in the connected mapped space")
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _request_short_path(self, target):
        if not self.path_client.server_is_ready():
            self.get_logger().warning("Waiting for the path planner before selecting a short step")
            return
        request = ComputePathToPose.Goal()
        request.goal = PoseStamped()
        request.goal.header.frame_id = self.map_frame
        request.goal.header.stamp = self.get_clock().now().to_msg()
        request.goal.pose.position.x, request.goal.pose.position.y = target[:2]
        navigation_yaw = target[2]
        if self.returning_to_start and self.align_after_arrival:
            rx, ry = self._robot_pose(self.map_frame)
            navigation_yaw = math.atan2(target[1]-ry, target[0]-rx)
        request.goal.pose.orientation.z = math.sin(navigation_yaw/2)
        request.goal.pose.orientation.w = math.cos(navigation_yaw/2)
        request.planner_id = 'GridBased'
        request.use_start = False
        self.path_pending = True
        self.goal_started_at = time.monotonic()
        self.path_target = target
        self.status_pub.publish(String(data="PLANNING_STEP: selecting a short waypoint on a Nav2 path"))
        self.path_client.send_goal_async(request).add_done_callback(self._on_short_path_response)

    def _on_short_path_response(self, future):
        try:
            handle = future.result()
        except Exception as error:
            self._stop(f"Path request failed: {error}")
            return
        if self.stopped:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._short_path_failed("Planner rejected the path request")
            return
        self.path_handle = handle
        handle.get_result_async().add_done_callback(self._on_short_path_result)

    def _short_path_failed(self, reason):
        self.path_pending = False
        self.path_handle = None
        if self.returning_to_start:
            self._stop(reason)
            return
        self.failed_positions.append(self.path_target[:2])
        self.retry_after = time.monotonic()+2.0
        self.get_logger().warning(reason+"; selecting another reachable frontier")

    def _on_short_path_result(self, future):
        self.path_pending = False
        self.path_handle = None
        if self.stopped:
            return
        try:
            result = future.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.path.poses:
                self._short_path_failed("No path to the selected frontier")
                return
            path = result.result.path
            if path.header.frame_id != self.map_frame:
                raise ValueError("Planned path frame differs from the exploration frame")
            poses = []
            for item in path.poses:
                p, q = item.pose.position, item.pose.orientation
                poses.append((p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
            robot = self._robot_pose(self.map_frame)
            x, y, yaw, final = short_path_goal(poses, robot, self.maximum_step_distance)
            if self.returning_to_start:
                self.return_step_is_final = final and math.hypot(
                    self.start_pose[0]-robot[0], self.start_pose[1]-robot[1]) <= self.maximum_step_distance
            clearance, size = self.path_target[3:]
            self.get_logger().info(f"Short step: ({x:.2f}, {y:.2f}); planned path step <= {self.maximum_step_distance:.2f}m")
            self._publish_markers(self.map_msg, [], (x, y, yaw, clearance, size))
            arrival_yaw = self.start_pose[2] if self.returning_to_start and self.return_step_is_final else yaw
            self._send_navigation_goal(x, y, yaw, clearance, size, arrival_yaw=arrival_yaw)
        except Exception as error:
            self._short_path_failed(f"Could not select a short path waypoint: {error}")

    def _orient_after_arrival(self):
        if not getattr(self, "align_after_arrival", False):
            self._finish_arrival()
            return
        try:
            self._robot_pose(self.map_frame)
            angle = math.atan2(math.sin(self.current_goal_yaw-self.robot_yaw),
                               math.cos(self.current_goal_yaw-self.robot_yaw))
        except TransformException:
            self._stop("Pose unavailable for final in-place rotation")
            return
        if abs(angle) <= 0.25:
            self._finish_arrival()
            return
        if not self.orientation_client.server_is_ready():
            self._stop("Spin server unavailable for final in-place rotation")
            return
        goal = Spin.Goal()
        goal.target_yaw = angle
        goal.time_allowance = Duration(sec=40)
        self.goal_pending = True
        self.goal_started_at = time.monotonic()
        self.get_logger().info(f"Position reached; turning in place {math.degrees(angle):.1f} degrees")
        self.orientation_client.send_goal_async(goal).add_done_callback(self._on_orientation_response)

    def _on_orientation_response(self, future):
        self.goal_pending = False
        try:
            handle = future.result()
            if self.stopped:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self._stop("Final in-place rotation was rejected")
                return
            self.orientation_handle = handle
            self.goal_active = True
            self._nav2_goal_active = True
            if not self._set_command_source("NAV2", "aligning heading after position arrival"):
                self._stop("Command selector unavailable for final rotation")
                return
            self.status_pub.publish(String(data="ORIENTING: position reached; rotating in place"))
            handle.get_result_async().add_done_callback(self._on_orientation_result)
        except Exception as error:
            self._stop(f"Could not start final rotation: {error}")

    def _on_orientation_result(self, future):
        self.goal_active = False
        self.orientation_handle = None
        self._nav2_goal_active = False
        self._set_command_source("STOP", "final in-place rotation finished")
        if self.stopped:
            return
        try:
            if future.result().status != GoalStatus.STATUS_SUCCEEDED:
                self._stop("Final in-place rotation did not succeed")
                return
        except Exception as error:
            self._stop(f"Final rotation result failed: {error}")
            return
        self._finish_arrival()

    def _finish_arrival(self):
        if self.returning_to_start:
            if not getattr(self, "return_step_is_final", True):
                self.return_retry_pending = True
                return
            self.returning_to_start = False
            self._complete(returned_to_start=True)
            return
        self.successful_goal_count = getattr(self, "successful_goal_count", 0)+1
        if getattr(self, "current_goal_xy", None) is not None:
            self.visited_positions.append(self.current_goal_xy)
        self.get_logger().info(f"Frontier {self.goal_number} reached; selecting from updated map")

    def _return_to_start(self):
        if not self.return_to_start or self.start_pose is None:
            self._complete()
            return
        if not self.action_client.server_is_ready():
            self.get_logger().warning("NavigateToPose is not ready; waiting before return to start")
            self.no_frontier_reported = False
            return

        x, y, yaw = self.start_pose
        self.returning_to_start = True
        self.return_step_is_final = True
        try:
            rx, ry = self._robot_pose(self.map_frame)
        except TransformException:
            self._stop("Current pose unavailable before returning home")
            return
        if math.hypot(x-rx, y-ry) > getattr(self, "maximum_step_distance", math.inf):
            self._request_short_path((x, y, yaw, self.minimum_clearance, 0))
            return
        goal = NavigateToPose.Goal()
        goal.behavior_tree = getattr(self, "step_behavior_tree", "")
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        navigation_yaw = math.atan2(y-ry, x-rx) if getattr(self, "align_after_arrival", False) else yaw
        pose.pose.orientation.z = math.sin(navigation_yaw * 0.5)
        pose.pose.orientation.w = math.cos(navigation_yaw * 0.5)
        goal.pose = pose

        self.returning_to_start = True
        self.current_goal_xy = (x, y)
        self.current_goal_yaw = yaw
        self.goal_pending = True
        self.goal_started_at = time.monotonic()
        self.get_logger().info(
            f"Returning to exploration start: ({x:.2f}, {y:.2f}), yaw={yaw:.2f}"
        )
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _start_collision_backup(self, resume_return=False):
        if not self.backup_client.server_is_ready():
            self._stop("Nav2 BackUp recovery is unavailable; operator inspection required")
            return
        self.retry_return_after_backup = resume_return
        self.backup_attempt = 0
        self._send_collision_backup()

    def _send_collision_backup(self):
        goal = BackUp.Goal()
        distance = max(self.backup_distance * (0.5 ** self.backup_attempt), 0.05)
        goal.target.x = distance
        goal.target.y = 0.0
        goal.target.z = 0.0
        goal.speed = self.backup_speed
        goal.time_allowance = Duration(
            sec=int(self.backup_time_allowance),
            nanosec=int((self.backup_time_allowance % 1.0) * 1e9),
        )
        self.goal_pending = True
        self.goal_started_at = time.monotonic()
        self.get_logger().warning(
            f"Collision recovery: backing up {distance:.2f} m "
            f"(attempt {self.backup_attempt + 1}/3) at {self.backup_speed:.2f} m/s"
        )
        self.status_pub.publish(String(data="BACKING_UP: checking reverse path with Nav2 costmap"))
        future = self.backup_client.send_goal_async(goal)
        future.add_done_callback(self._on_backup_goal_response)

    def _on_backup_goal_response(self, future):
        self.goal_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            self._stop(f"Nav2 BackUp request failed: {exc}")
            return
        if self.stopped:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._stop("Nav2 rejected collision recovery backup")
            return
        self.backup_handle = handle
        self.goal_active = True
        self._nav2_goal_active = True
        if not self._set_command_source("NAV2", "collision recovery backup accepted"):
            self._stop("Command selector unavailable for collision recovery")
            return
        handle.get_result_async().add_done_callback(self._on_backup_goal_result)

    def _on_backup_goal_result(self, future):
        self.goal_active = False
        self.goal_pending = False
        self.backup_handle = None
        self._nav2_goal_active = False
        if not self.dry_run:
            self._set_command_source("STOP", "collision recovery backup finished")
        try:
            result = future.result()
        except Exception as exc:
            self._stop(f"Nav2 BackUp result failed: {exc}")
            return
        if self.stopped:
            return
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            if self.backup_attempt < 2:
                self.backup_attempt += 1
                self.get_logger().warning(
                    "Reverse path blocked at this distance; retrying a shorter costmap-checked backup"
                )
                self._send_collision_backup()
                return
            if self.retry_return_after_backup:
                self.retry_return_after_backup = False
                self._stop("Reverse path blocked; cannot safely retry the return goal")
                return
            if self.stop_after_backup:
                self._stop("Three navigation failures; reverse path blocked and operator inspection is required")
                return
            self.retry_after = time.monotonic() + 2.0
            self.status_pub.publish(String(
                data="REPLANNING: reverse path blocked; trying a different frontier from current pose"
            ))
            self.get_logger().warning(
                "Reverse path remains blocked at 0.05 m; trying another frontier from the current pose"
            )
            return

        self.get_logger().info("Backup succeeded; updating pose and selecting a new route")
        if self.stop_after_backup:
            self._stop("Three navigation failures; stopped after safe backup for operator inspection")
            return
        if self.retry_return_after_backup:
            self.retry_return_after_backup = False
            self.return_retry_pending = True
        else:
            self.retry_after = time.monotonic() + 2.0
            self.status_pub.publish(String(data="REPLANNING: backup complete; choosing another frontier"))

    def _tick(self):
        if self.stopped or self.map_msg is None:
            return
        if getattr(self, "path_pending", False):
            if time.monotonic()-self.goal_started_at > 15.0:
                self._stop("Timed out computing a short navigation step")
            return
        if time.monotonic() < getattr(self, "retry_after", 0.):
            return
        if getattr(self, "return_retry_pending", False):
            self.return_retry_pending = False
            self._return_to_start()
            return
        if self.map_frame and self.map_msg.header.frame_id != self.map_frame:
            self._stop("Map frame changed; restart exploration")
            return
        if self.goal_pending or self.goal_active:
            if time.monotonic() - self.map_received_at > self.map_stale_sec:
                reason = ("Map became stale during return to start" if self.returning_to_start
                          else "Map became stale during navigation")
                self._stop(reason)
            elif time.monotonic() - self.goal_started_at > self.goal_timeout:
                reason = ("Return-to-start navigation timed out" if self.returning_to_start
                          else "Frontier navigation timed out")
                self._stop(reason)
            else:
                try:
                    self._robot_pose(self.map_frame)
                except TransformException:
                    self._stop("Robot transform unavailable during navigation")
            return
        if time.monotonic() - self.map_received_at > self.map_stale_sec:
            self.get_logger().warning("DistanceMapSlice is stale; waiting for a fresh map")
            return
        msg = self.map_msg
        if not msg.header.frame_id:
            self.get_logger().error("DistanceMapSlice has no frame_id; exploration stopped")
            self.stopped = True
            return
        try:
            robot_x, robot_y = self._robot_pose(msg.header.frame_id)
        except TransformException as exc:
            self.get_logger().warning(f"Waiting for robot pose in {msg.header.frame_id}: {exc}")
            return

        if self.area_center is None:
            self.area_center = (robot_x, robot_y)
            self.start_pose = (robot_x, robot_y, self.robot_yaw)
            self.area_yaw = self.robot_yaw if self.area_align_with_start_heading else 0.0
            if not any(math.hypot(robot_x-x, robot_y-y) < 0.10
                       for x, y in self.visited_positions):
                self.visited_positions.append((robot_x, robot_y))
            self.map_frame = msg.header.frame_id
            self.get_logger().info(
                f"Exploration start fixed at ({robot_x:.2f}, {robot_y:.2f}, "
                f"yaw={math.degrees(self.area_yaw):.1f} deg) in {msg.header.frame_id}; "
                f"area={self.area_height:.1f}m forward x {self.area_width:.1f}m wide, "
                "start at rear-left corner"
            )

        clusters = self._unvisited_frontiers(
            self._find_frontier_clusters(msg, robot_x, robot_y))
        self.goal_kind = "FRONTIER"
        if not clusters:
            self.goal_kind = "OBSERVATION"
            observed = frontier_clusters(
                msg, (robot_x, robot_y), self.area_center, self.area_radius,
                self.minimum_clearance, self.minimum_goal_distance, 1,
                self.area_width, self.area_height, self.area_offset_x, self.area_offset_y,
                include_observed=True, connection_clearance=self.connection_clearance,
                stats=self.search_stats, area_yaw=self.area_yaw)
            clusters = [[p for p in cluster if all(
                math.hypot(p[0]-vx, p[1]-vy) >= 0.75
                for vx, vy in self.visited_positions)] for cluster in observed]
            clusters = [cluster for cluster in clusters if cluster]
        goal, cluster_count = self._choose_goal(msg, robot_x, robot_y, clusters)
        self._publish_markers(msg, clusters, goal)
        if goal is None:
            if self.no_frontier_since is None:
                self.no_frontier_since = time.monotonic()
            elif (not self.no_frontier_reported and
                  time.monotonic() - self.no_frontier_since >= self.no_frontier_confirm_sec):
                stats = getattr(self, "search_stats", {})
                if (not self.dry_run and stats.get("reachable_cells", 0) > 0
                        and stats.get("eligible_cells", 0) > 0):
                    self.no_frontier_reported = True
                    self._return_to_start()
                    return
                self.no_frontier_reported = True
                self.status_pub.publish(String(data="WAITING: no eligible reachable frontier; observe more of the room"))
                self.get_logger().info(
                    "No eligible reachable frontier or unvisited observation point; waiting for more observations"
                )
            return

        self.no_frontier_since = None
        self.no_frontier_reported = False
        x, y, yaw, clearance, cluster_size = goal
        if self.dry_run:
            point = (round(x, 2), round(y, 2))
            if point != self.last_dry_goal:
                self.last_dry_goal = point
                self.get_logger().info(
                    f"DRY RUN {self.goal_kind} target=({x:.2f}, {y:.2f}), "
                    f"yaw={yaw:.2f}, clearance={clearance:.2f} m, "
                    f"cluster={cluster_size} cells, clusters={cluster_count}"
                )
            self.status_pub.publish(String(data="PREVIEW: reachable frontier selected"))
            return

        if not self.action_client.server_is_ready():
            self.get_logger().warning("NavigateToPose server is not ready; waiting")
            return

        if math.hypot(x-robot_x, y-robot_y) > getattr(self, "maximum_step_distance", math.inf):
            self._request_short_path(goal)
            return
        self._send_navigation_goal(x, y, yaw, clearance, cluster_size)

    def _send_navigation_goal(self, x, y, yaw, clearance, cluster_size, arrival_yaw=None):
        goal_msg = NavigateToPose.Goal()
        goal_msg.behavior_tree = getattr(self, "step_behavior_tree", "")
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        goal_msg.pose = pose

        self.current_goal_xy = (x, y)
        self.current_goal_yaw = yaw if arrival_yaw is None else arrival_yaw
        self.goal_number += 1
        self.goal_pending = True
        self.goal_started_at = time.monotonic()
        self.get_logger().info(
            f"Frontier {self.goal_number}: target=({x:.2f}, {y:.2f}), "
            f"yaw={yaw:.2f}, clearance={clearance:.2f} m, "
            f"cluster={cluster_size} cells, radius="
            f"{math.hypot(x - self.area_center[0], y - self.area_center[1]):.2f} m"
        )
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        self.goal_pending = False
        try:
            handle = future.result()
        except Exception as exc:  # action transport failure: don't keep exploring
            self.get_logger().error(f"NavigateToPose request failed: {exc}; exploration stopped")
            self._stop("Navigation request failed")
            return
        if self.stopped:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            reason = ("Nav2 rejected return-to-start goal" if self.returning_to_start
                      else "Nav2 rejected frontier goal")
            self._stop(reason)
            return
        self.goal_handle = handle
        self.goal_active = True
        self._nav2_goal_active = True
        if not self._set_command_source("NAV2", "own frontier goal accepted"):
            self._stop("Command selector unavailable")
            return
        if self.returning_to_start:
            self.status_pub.publish(String(data="RETURNING_TO_START: goal accepted by Nav2"))
            self.get_logger().info("Return-to-start goal accepted by Nav2")
        else:
            self.status_pub.publish(String(data=f"NAVIGATING: frontier {self.goal_number}"))
            self.get_logger().info(f"Frontier {self.goal_number} accepted by Nav2")
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self.goal_active = False
        self.goal_handle = None
        self._nav2_goal_active = False
        if not self.dry_run:
            self._set_command_source("STOP", "frontier goal finished")
        try:
            result = future.result()
        except Exception as exc:
            self._stop(f"Frontier result failed: {exc}")
            return
        if self.stopped:
            return
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._orient_after_arrival()
        elif result.status == GoalStatus.STATUS_ABORTED:
            if self.returning_to_start:
                self.return_recovery_attempts += 1
                if self.return_recovery_attempts > 1:
                    self.returning_to_start = False
                    self._stop("Return to exploration start failed after one backup; operator inspection required")
                    return
                self._start_collision_backup(resume_return=True)
                return
            self.navigation_failures = getattr(self, "navigation_failures", 0) + 1
            try:
                position = self._robot_pose(self.map_frame)
            except TransformException:
                self._stop("Navigation failed and current pose is unavailable")
                return
            # Keep actual observation coverage separately from action success.
            self.visited_positions.append(position)
            if self.current_goal_xy is not None:
                self.failed_positions.append(self.current_goal_xy)
            self.stop_after_backup = self.navigation_failures >= 3
            self._start_collision_backup()
        else:
            self._stop(f"Navigation ended with status {result.status}")



def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._stop("Explorer shutting down")
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
