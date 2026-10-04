#!/usr/bin/env python3
"""Greedy frontier exploration bounded around the starting pose.

Frontiers are detected directly in nvblox's DistanceMapSlice so unobserved
cells remain distinguishable from free cells in Nav2's merged costmap.
"""

from collections import deque
import math
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nvblox_msgs.msg import DistanceMapSlice
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import SetParameters
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


NEIGHBORS = (
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1),            (0, 1),
    (1, -1),  (1, 0),   (1, 1),
)


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        self.declare_parameter("map_topic", "/nvblox_node/static_map_slice")
        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("area_radius_m", 2.0)
        self.declare_parameter("minimum_clearance_m", 0.36)
        self.declare_parameter("minimum_goal_distance_m", 0.40)
        self.declare_parameter("minimum_cluster_cells", 5)
        self.declare_parameter("no_frontier_confirm_sec", 5.0)
        self.declare_parameter("map_stale_sec", 2.0)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("enable_selector_on_nav2_goal", True)

        self.area_radius = float(self.get_parameter("area_radius_m").value)
        self.minimum_clearance = float(self.get_parameter("minimum_clearance_m").value)
        self.minimum_goal_distance = float(self.get_parameter("minimum_goal_distance_m").value)
        self.minimum_cluster_cells = int(self.get_parameter("minimum_cluster_cells").value)
        self.no_frontier_confirm_sec = float(
            self.get_parameter("no_frontier_confirm_sec").value
        )
        self.map_stale_sec = float(self.get_parameter("map_stale_sec").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.enable_selector_on_nav2_goal = bool(
            self.get_parameter("enable_selector_on_nav2_goal").value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_msg = None
        self.map_received_at = 0.0
        self.area_center = None
        self.goal_pending = False
        self.goal_active = False
        self.stopped = False
        self.no_frontier_since = None
        self.goal_number = 0
        self.last_dry_goal = None
        self._nav2_goal_active = False
        self._selector_enable_requested = False
        self._goal_status_initialized = False
        self._seen_nav2_goal_ids = set()
        self._controlled_nav2_goal_ids = set()
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
        self.selector_parameter_client = self.create_client(
            SetParameters, "/leader/command_selector/set_parameters"
        )
        self.goal_status_sub = self.create_subscription(
            GoalStatusArray,
            str(self.get_parameter("action_name").value) + "/_action/status",
            self._on_nav2_goal_status,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.timer = self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f"Frontier exploration ready: radius={self.area_radius:.2f} m, "
            f"clearance={self.minimum_clearance:.2f} m, dry_run={self.dry_run}"
        )

    def _on_map(self, msg):
        self.map_msg = msg
        self.map_received_at = time.monotonic()

    def _on_nav2_goal_status(self, msg):
        active_statuses = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
        }
        terminal_statuses = {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        }
        entries = [
            (bytes(item.goal_info.goal_id.uuid), int(item.status))
            for item in msg.status_list
        ]
        if not self._goal_status_initialized:
            self._seen_nav2_goal_ids.update(goal_id for goal_id, _ in entries)
            self._goal_status_initialized = True
            return

        new_active_goal_ids = {
            goal_id
            for goal_id, status in entries
            if status in active_statuses and goal_id not in self._seen_nav2_goal_ids
        }
        self._seen_nav2_goal_ids.update(goal_id for goal_id, _ in entries)
        terminal_goal_ids = {
            goal_id for goal_id, status in entries if status in terminal_statuses
        }

        self._controlled_nav2_goal_ids.update(new_active_goal_ids)
        self._controlled_nav2_goal_ids.difference_update(terminal_goal_ids)
        self._nav2_goal_active = bool(self._controlled_nav2_goal_ids)
        if new_active_goal_ids:
            self._selector_enable_requested = False
        if self._nav2_goal_active and self.enable_selector_on_nav2_goal:
            if not self._selector_enable_requested:
                self._selector_enable_requested = self._set_command_source(
                    "NAV2", "new Nav2 goal became active"
                )
        elif terminal_goal_ids and not self._nav2_goal_active:
            self._selector_enable_requested = False

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
            return
        if not results or not results[0].successful:
            detail = results[0].reason if results else "no parameter result"
            self.get_logger().error(
                f"Command selector refused {source}: {detail}"
            )
            self._selector_enable_requested = False
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
        p = transform.transform.translation
        return float(p.x), float(p.y)

    @staticmethod
    def _unknown(value, sentinel):
        return value == sentinel or (math.isnan(value) and math.isnan(sentinel))

    def _find_frontier_clusters(self, msg, robot_x, robot_y):
        width, height = int(msg.width), int(msg.height)
        resolution = float(msg.resolution)
        data = msg.data
        if resolution <= 0.0 or width < 3 or height < 3 or len(data) != width * height:
            return []

        values = [float(value) for value in data]

        def value_at(row, col):
            return values[row * width + col]

        def cell_xy(row, col):
            return (
                float(msg.origin.x) + (col + 0.5) * resolution,
                float(msg.origin.y) + (row + 0.5) * resolution,
            )

        candidates = {}
        radius_sq = self.area_radius * self.area_radius
        unknown_value = float(msg.unknown_value)
        for row in range(1, height - 1):
            for col in range(1, width - 1):
                distance = value_at(row, col)
                if (
                    self._unknown(distance, unknown_value)
                    or not math.isfinite(distance)
                    or distance < self.minimum_clearance
                ):
                    continue
                x, y = cell_xy(row, col)
                if (x - self.area_center[0]) ** 2 + (y - self.area_center[1]) ** 2 > radius_sq:
                    continue

                unknown_points = []
                for dr, dc in NEIGHBORS:
                    neighbor = value_at(row + dr, col + dc)
                    if self._unknown(neighbor, unknown_value):
                        unknown_points.append(cell_xy(row + dr, col + dc))
                if not unknown_points:
                    continue

                ux = sum(point[0] for point in unknown_points) / len(unknown_points)
                uy = sum(point[1] for point in unknown_points) / len(unknown_points)
                if math.hypot(x - robot_x, y - robot_y) < self.minimum_goal_distance:
                    continue
                candidates[(row, col)] = (x, y, distance, ux, uy)

        clusters = []
        unseen = set(candidates)
        while unseen:
            seed = unseen.pop()
            queue = deque([seed])
            cluster = [seed]
            while queue:
                row, col = queue.popleft()
                for dr, dc in NEIGHBORS:
                    adjacent = (row + dr, col + dc)
                    if adjacent in unseen:
                        unseen.remove(adjacent)
                        queue.append(adjacent)
                        cluster.append(adjacent)
            if len(cluster) >= self.minimum_cluster_cells:
                clusters.append([candidates[cell] for cell in cluster])
        return clusters

    def _choose_goal(self, msg, robot_x, robot_y):
        clusters = self._find_frontier_clusters(msg, robot_x, robot_y)
        best = None
        best_score = math.inf
        for cluster in clusters:
            mean_x = sum(item[0] for item in cluster) / len(cluster)
            mean_y = sum(item[1] for item in cluster) / len(cluster)
            candidate = min(
                cluster,
                key=lambda item: (
                    (item[0] - mean_x) ** 2 + (item[1] - mean_y) ** 2,
                    -item[2],
                ),
            )
            x, y, clearance, unknown_x, unknown_y = candidate
            travel = math.hypot(x - robot_x, y - robot_y)
            score = travel - 0.02 * math.sqrt(len(cluster))
            if score < best_score:
                best_score = score
                best = (x, y, math.atan2(unknown_y - y, unknown_x - x), clearance, len(cluster))
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
            for step in range(65):
                angle = 2.0 * math.pi * step / 64.0
                point = Point()
                point.x = self.area_center[0] + self.area_radius * math.cos(angle)
                point.y = self.area_center[1] + self.area_radius * math.sin(angle)
                point.z = 0.03
                boundary.points.append(point)
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

    def _tick(self):
        if self.stopped or self.goal_pending or self.goal_active or self.map_msg is None:
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
            self.get_logger().info(
                f"Exploration center fixed at ({robot_x:.2f}, {robot_y:.2f}) "
                f"in {msg.header.frame_id}"
            )

        clusters = self._find_frontier_clusters(msg, robot_x, robot_y)
        goal, cluster_count = self._choose_goal(msg, robot_x, robot_y)
        self._publish_markers(msg, clusters, goal)
        if goal is None:
            if self.no_frontier_since is None:
                self.no_frontier_since = time.monotonic()
            elif time.monotonic() - self.no_frontier_since >= self.no_frontier_confirm_sec:
                self.get_logger().info(
                    f"No reachable frontiers remain within {self.area_radius:.2f} m; "
                    "exploration complete"
                )
            return

        self.no_frontier_since = None
        x, y, yaw, clearance, cluster_size = goal
        if self.dry_run:
            point = (round(x, 2), round(y, 2))
            if point != self.last_dry_goal:
                self.last_dry_goal = point
                self.get_logger().info(
                    f"DRY RUN frontier target=({x:.2f}, {y:.2f}), "
                    f"yaw={yaw:.2f}, clearance={clearance:.2f} m, "
                    f"cluster={cluster_size} cells, clusters={cluster_count}"
                )
                self.stopped = True
            return

        if not self.action_client.server_is_ready():
            self.get_logger().warning("NavigateToPose server is not ready; waiting")
            return

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        goal_msg.pose = pose

        self.goal_number += 1
        self.goal_pending = True
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
            self.stopped = True
            return
        if not handle.accepted:
            self.get_logger().error("Nav2 rejected frontier goal; exploration stopped")
            self.stopped = True
            return
        self.goal_active = True
        self.get_logger().info(f"Frontier {self.goal_number} accepted by Nav2")
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self.goal_active = False
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Frontier result failed: {exc}; exploration stopped")
            self.stopped = True
            return
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Frontier {self.goal_number} reached; selecting from updated map")
        else:
            self.get_logger().error(
                f"Frontier {self.goal_number} ended with Nav2 status {result.status}; "
                "exploration stopped"
            )
            self.stopped = True


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
