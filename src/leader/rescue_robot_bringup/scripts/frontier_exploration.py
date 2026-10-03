#!/usr/bin/env python3
"""Explore a disk around a point two metres ahead of the startup pose."""

import math
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from nvblox_msgs.msg import DistanceMapSlice
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class FrontierExploration(Node):
    def __init__(self):
        super().__init__("frontier_exploration")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("odometry_topic", "/leader/odom/raw")
        self.declare_parameter("target_offset", 2.0)
        self.declare_parameter("exploration_radius", 2.0)
        self.declare_parameter("center_override_enabled", False)
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("max_cost", 70)
        self.declare_parameter("minimum_obstacle_clearance", 0.35)
        self.declare_parameter("frontier_min_cells", 2)
        self.declare_parameter("goal_separation", 0.35)
        self.declare_parameter("rescan_period", 1.0)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        slice_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.VOLATILE)
        self.grid: Optional[OccupancyGrid] = None
        self.distance_slice: Optional[DistanceMapSlice] = None
        self.odom: Optional[Odometry] = None
        self.center = None
        self.visited = []
        self.active_goal = None
        self.goal_active = False
        self.finished = False
        self.last_scan = 0.0
        self.last_costmap_receipt = None
        self.last_slice_receipt = None
        self.last_odom_receipt = None
        self.input_counts = {"costmap": 0, "slice": 0, "wheel_odom": 0}
        self.last_health_counts = dict(self.input_counts)
        self.last_health_log = 0.0
        self.last_readiness_log = 0.0
        self.goal_started_at = None
        self.last_feedback_log = 0.0
        if bool(self.get_parameter("center_override_enabled").value):
            self.center = (float(self.get_parameter("center_x").value),
                           float(self.get_parameter("center_y").value))
            self.get_logger().info(
                f"Using exploration disk center in odom: ({self.center[0]:.2f}, "
                f"{self.center[1]:.2f}), radius "
                f"{self.get_parameter('exploration_radius').value:.2f} m")
        self.goal_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.create_subscription(OccupancyGrid, self.get_parameter("costmap_topic").value,
                                 self.on_grid, qos)
        self.create_subscription(DistanceMapSlice, "/nvblox_node/static_map_slice",
                                 self.on_distance_slice, slice_qos)
        self.create_subscription(Odometry, self.get_parameter("odometry_topic").value,
                                 self.on_odom, 10)
        self.create_timer(0.2, self.tick)
        self.get_logger().info(
            "Waiting for costmap, wheel odometry, nvblox distance slice, and Nav2 action server")

    def on_grid(self, msg):
        self.grid = msg
        self.input_counts["costmap"] += 1
        self.last_costmap_receipt = self.get_clock().now().nanoseconds / 1e9

    def on_distance_slice(self, msg):
        self.distance_slice = msg
        self.input_counts["slice"] += 1
        self.last_slice_receipt = self.get_clock().now().nanoseconds / 1e9

    def on_odom(self, msg):
        self.odom = msg
        self.input_counts["wheel_odom"] += 1
        self.last_odom_receipt = self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _message_stamp_seconds(message):
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        if stamp is None:
            return None
        return stamp.sec + stamp.nanosec * 1e-9

    def log_input_health(self, now):
        if now - self.last_health_log < 5.0:
            return
        self.last_health_log = now
        count_delta = {
            name: count - self.last_health_counts[name]
            for name, count in self.input_counts.items()
        }
        self.last_health_counts = dict(self.input_counts)

        def age(receipt):
            return now - receipt if receipt is not None else float("inf")

        costmap_age = age(self.last_costmap_receipt)
        slice_age = age(self.last_slice_receipt)
        odom_age = age(self.last_odom_receipt)
        grid_summary = "missing"
        if self.grid is not None:
            grid = self.grid
            data = grid.data
            unknown = sum(value < 0 for value in data)
            lethal = sum(value >= 100 for value in data)
            grid_summary = (
                f"frame={grid.header.frame_id} size={grid.info.width}x{grid.info.height} "
                f"res={grid.info.resolution:.3f} unknown={unknown} lethal={lethal}"
            )
        pose_summary = "missing"
        if self.odom is not None:
            pose = self.odom.pose.pose
            twist = self.odom.twist.twist
            pose_summary = (
                f"pose=({pose.position.x:.3f},{pose.position.y:.3f}) "
                f"twist=({twist.linear.x:.3f},{twist.angular.z:.3f})"
            )
        self.get_logger().info(
            "Input health counts_5s=%s totals=%s receipt_age(costmap/slice/odom)="
            "%.2f/%.2f/%.2f s costmap_stamp=%.3f slice_stamp=%.3f odom_stamp=%.3f "
            "%s %s" % (
                count_delta, self.input_counts, costmap_age, slice_age, odom_age,
                self._message_stamp_seconds(self.grid) or 0.0,
                self._message_stamp_seconds(self.distance_slice) or 0.0,
                self._message_stamp_seconds(self.odom) or 0.0,
                grid_summary, pose_summary))
        stale = [f"{name}={value:.2f}s" for name, value in (
            ("costmap", costmap_age), ("slice", slice_age), ("wheel_odom", odom_age))
            if value > 2.0]
        if stale:
            self.get_logger().warning("Navigation input stale: %s" % ", ".join(stale))

    def tick(self):
        if self.finished:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        self.log_input_health(now)
        if self.goal_active:
            return
        if self.grid is None or self.distance_slice is None or self.odom is None:
            if now - self.last_readiness_log >= 5.0:
                self.last_readiness_log = now
                waiting = []
                if self.grid is None:
                    waiting.append("costmap")
                if self.distance_slice is None:
                    waiting.append("nvblox_slice")
                if self.odom is None:
                    waiting.append("wheel_odom")
                self.get_logger().warning("Waiting for required inputs: %s" % ",".join(waiting))
            return
        if not self.goal_client.server_is_ready():
            if now - self.last_readiness_log >= 5.0:
                self.last_readiness_log = now
                self.get_logger().warning("Nav2 NavigateToPose action server is not ready")
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_scan < float(self.get_parameter("rescan_period").value):
            return
        self.last_scan = now
        if self.center is None:
            p = self.odom.pose.pose.position
            q = self.odom.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            offset = float(self.get_parameter("target_offset").value)
            self.center = (p.x + offset * math.cos(yaw), p.y + offset * math.sin(yaw))
            self.get_logger().info(
                f"Exploration disk center in {self.grid.header.frame_id}: "
                f"({self.center[0]:.2f}, {self.center[1]:.2f}), "
                f"radius {self.get_parameter('exploration_radius').value:.2f} m")
        candidate = self.choose_frontier()
        if candidate is None:
            self.finished = True
            self.get_logger().info("No unvisited reachable frontier remains inside the exploration disk")
            return
        self.send_goal(*candidate)

    def choose_frontier(self):
        grid = self.grid
        distance_map = self.distance_slice
        width, height = grid.info.width, grid.info.height
        resolution = grid.info.resolution
        if (width < 3 or height < 3 or len(grid.data) != width * height
                or resolution <= 0 or distance_map.resolution <= 0):
            self.get_logger().error(
                "Invalid map geometry: costmap=%dx%d res=%.4f data=%d slice=%dx%d res=%.4f" % (
                    width, height, resolution, len(grid.data), distance_map.width,
                    distance_map.height, distance_map.resolution))
            return None
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        radius = float(self.get_parameter("exploration_radius").value)
        max_cost = int(self.get_parameter("max_cost").value)
        min_cells = int(self.get_parameter("frontier_min_cells").value)
        clearance = float(self.get_parameter("minimum_obstacle_clearance").value)
        robot = self.odom.pose.pose.position
        candidates = []
        counts = {"free_cost": 0, "inside_disk": 0, "known_clearance": 0,
                  "clearance_ok": 0, "frontier": 0, "unvisited": 0}
        # Sample frontier cells. Clustering adjacent cells avoids sending a goal
        # for every pixel while keeping the chosen point on known free space.
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                index = y * width + x
                if grid.data[index] < 0 or grid.data[index] > max_cost:
                    continue
                counts["free_cost"] += 1
                wx = ox + (x + 0.5) * resolution
                wy = oy + (y + 0.5) * resolution
                if math.hypot(wx - self.center[0], wy - self.center[1]) > radius:
                    continue
                counts["inside_disk"] += 1
                dm_x = int((wx - distance_map.origin.x) / distance_map.resolution)
                dm_y = int((wy - distance_map.origin.y) / distance_map.resolution)
                if not (0 < dm_x < distance_map.width - 1
                        and 0 < dm_y < distance_map.height - 1):
                    continue
                dm_index = dm_y * distance_map.width + dm_x
                distance = distance_map.data[dm_index]
                if abs(distance - distance_map.unknown_value) < 1e-3:
                    continue
                counts["known_clearance"] += 1
                if distance < clearance:
                    continue
                counts["clearance_ok"] += 1
                slice_width = distance_map.width
                if not any(
                    abs(distance_map.data[(dm_y + dy) * slice_width + dm_x + dx]
                        - distance_map.unknown_value) < 1e-3
                    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
                ):
                    continue
                counts["frontier"] += 1
                if any(math.hypot(wx - vx, wy - vy) <
                       float(self.get_parameter("goal_separation").value)
                       for vx, vy in self.visited):
                    continue
                counts["unvisited"] += 1
                score = math.hypot(wx - robot.x, wy - robot.y) \
                    + 0.15 * math.hypot(wx - self.center[0], wy - self.center[1])
                candidates.append((score, wx, wy, grid.data[index], distance))
        self.get_logger().info(
            "Frontier scan frame=%s size=%dx%d res=%.3f origin=(%.2f,%.2f) "
            "counts=%s candidates=%d" % (
                grid.header.frame_id, width, height, resolution, ox, oy,
                counts, len(candidates)))
        if len(candidates) < min_cells:
            return None
        # Prefer nearby frontiers, with a slight preference toward the disk center.
        selected = min(candidates, key=lambda item: item[0])
        _, x, y, cost, clearance_value = selected
        self.get_logger().info(
            "Selected frontier=(%.2f,%.2f) cost=%d esdf_clearance=%.3f "
            "robot=(%.2f,%.2f) distance=%.3f score=%.3f" % (
                x, y, cost, clearance_value, robot.x, robot.y,
                math.hypot(x - robot.x, y - robot.y), selected[0]))
        return x, y

    def send_goal(self, x, y):
        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = self.grid.header.frame_id
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        p = self.odom.pose.pose.position
        goal_yaw = math.atan2(y - p.y, x - p.x)
        msg.pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(goal_yaw / 2.0)
        self.active_goal = (x, y)
        self.goal_active = True
        self.goal_started_at = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            "Sending goal=(%.3f,%.3f) from=(%.3f,%.3f) distance=%.3f "
            "yaw=%.1fdeg frame=%s" % (
                x, y, p.x, p.y, math.hypot(x - p.x, y - p.y),
                math.degrees(goal_yaw), msg.pose.header.frame_id))
        future = self.goal_client.send_goal_async(msg, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_feedback(self, feedback_message):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_feedback_log < 1.0:
            return
        self.last_feedback_log = now
        feedback = feedback_message.feedback
        remaining = getattr(feedback, "distance_remaining", float("nan"))
        recoveries = getattr(feedback, "number_of_recoveries", -1)
        pose = getattr(feedback, "current_pose", None)
        if pose is not None:
            p = pose.pose.position
            pose_text = "pose=(%.3f,%.3f)" % (p.x, p.y)
        else:
            pose_text = "pose=unavailable"
        self.get_logger().info(
            "Nav2 feedback remaining=%.3f recoveries=%d %s" %
            (remaining, recoveries, pose_text))

    def on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as error:
            self.get_logger().error("Nav2 goal response raised: %r" % error)
            self.goal_active = False
            self.finished = True
            return
        if not handle.accepted:
            self.get_logger().error(
                "Nav2 rejected goal=%s; stopping exploration" % (self.active_goal,))
            self.goal_active = False
            self.finished = True
            return
        result = handle.get_result_async()
        result.add_done_callback(self.on_result)

    def on_result(self, future):
        try:
            result = future.result()
        except Exception as error:
            self.get_logger().error("Nav2 result raised for goal=%s: %r" %
                                    (self.active_goal, error))
            self.goal_active = False
            self.finished = True
            return
        status = result.status
        now = self.get_clock().now().nanoseconds / 1e9
        duration = now - self.goal_started_at if self.goal_started_at is not None else float("nan")
        self.get_logger().info("Nav2 goal result=%d goal=%s duration=%.2fs" %
                               (status, self.active_goal, duration))
        self.goal_active = False
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.visited.append(self.active_goal)
            self.get_logger().info("Frontier goal reached")
            self.last_scan = 0.0
        else:
            self.get_logger().error(
                f"Frontier goal ended with status {status}; stopping for operator review")
            self.finished = True
        self.active_goal = None


def main():
    rclpy.init()
    node = FrontierExploration()
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
