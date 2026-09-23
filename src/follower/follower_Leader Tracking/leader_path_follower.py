#!/usr/bin/env python3

"""Follow the Leader odometry path after an explicit keyboard start command."""

from dataclasses import dataclass
import math
import os
import select
import sys
import termios
import time
import tty
from typing import List, Optional

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker


ARROW_KEYS = {
    "\x1b[A": "forward",
    "\x1b[B": "reverse",
    "\x1b[C": "right",
    "\x1b[D": "left",
}

COMBINATION_KEYS = {
    "q": "forward_left",
    "w": "forward_right",
    "a": "reverse_left",
    "s": "reverse_right",
}


@dataclass
class PlanarPose:
    x: float
    y: float
    yaw: float


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_odometry(message: Odometry) -> float:
    q = message.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class LeaderPathFollower(Node):
    """Record Leader motion and drive the Follower along the transformed path."""

    def __init__(self) -> None:
        super().__init__("leader_path_follower")

        self.declare_parameter("leader_odom_topic", "/leader/odom/raw")
        self.declare_parameter("follower_odom_topic", "/follower/odom/raw")
        self.declare_parameter("leader_command_topic", "/leader/cmd_vel")
        self.declare_parameter("command_topic", "/follower/cmd_vel")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("manual_linear_speed", 0.05)
        self.declare_parameter("manual_angular_speed", 0.15)
        self.declare_parameter("max_linear_speed", 0.05)
        self.declare_parameter("max_angular_speed", 0.15)
        self.declare_parameter("lookahead_distance", 0.25)
        self.declare_parameter("path_point_spacing", 0.02)
        self.declare_parameter("goal_tolerance", 0.04)
        self.declare_parameter("pose_timeout", 0.50)
        self.declare_parameter("leader_command_timeout", 0.50)
        self.declare_parameter("key_timeout", 0.25)
        self.declare_parameter("max_path_error", 0.40)
        self.declare_parameter("heading_gain", 0.8)
        self.declare_parameter("turn_in_place_angle", 1.05)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("maximum_path_points", 20000)
        self.declare_parameter("opposite_facing", True)

        if not sys.stdin.isatty():
            raise RuntimeError(
                "leader_path_follower must run with ros2 run in an interactive terminal"
            )

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._leader_pose: Optional[PlanarPose] = None
        self._follower_pose: Optional[PlanarPose] = None
        self._leader_command = Twist()
        self._leader_command_received_at = 0.0
        self._leader_received_at = 0.0
        self._follower_received_at = 0.0
        self._leader_start: Optional[PlanarPose] = None
        self._follower_start: Optional[PlanarPose] = None
        self._path: List[PlanarPose] = []
        self._follower_actual: List[PlanarPose] = []
        self._progress_index = 0
        self._active = False
        self._last_warning_at = 0.0
        self._input_buffer = ""
        self._manual_motion: Optional[str] = None
        self._last_manual_key_time = 0.0

        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._command_pub = self.create_publisher(
            Twist, str(self.get_parameter("command_topic").value), qos
        )
        self._leader_relative_odom_pub = self.create_publisher(
            Odometry, "/cooperation/leader/relative_odom", qos
        )
        self._follower_relative_odom_pub = self.create_publisher(
            Odometry, "/cooperation/follower/relative_odom", qos
        )
        self._leader_path_pub = self.create_publisher(
            Path, "/cooperation/paths/leader_reference", latched_qos
        )
        self._follower_path_pub = self.create_publisher(
            Path, "/cooperation/paths/follower_actual", latched_qos
        )
        self._target_pub = self.create_publisher(
            Marker, "/follower/follow/target_marker", latched_qos
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("leader_odom_topic").value),
            self._on_leader_odometry,
            qos,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("leader_command_topic").value),
            self._on_leader_command,
            qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("follower_odom_topic").value),
            self._on_follower_odometry,
            qos,
        )

        self._stdin_fd = sys.stdin.fileno()
        self._terminal_settings = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)
        self._terminal_configured = True

        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            "MANUAL mode. Arrows/QWAS: drive, P: AUTO, SPACE: stop, "
            "R: reset, Ctrl-C: exit"
        )
        self.get_logger().info(
            "Waiting for Leader/Follower odometry; motors remain stopped."
        )

    def _on_leader_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self._leader_pose = PlanarPose(
            float(position.x), float(position.y), yaw_from_odometry(message)
        )
        self._leader_received_at = time.monotonic()
        if self._leader_start is not None and self._follower_start is not None:
            self._append_leader_path(self._transform_leader_pose(self._leader_pose))
            self._publish_relative_odometry(
                self._leader_relative_odom_pub,
                self._leader_pose,
                self._leader_start,
                "leader_start",
                rotate_180=bool(self.get_parameter("opposite_facing").value),
            )

    def _on_leader_command(self, message: Twist) -> None:
        self._leader_command = message
        self._leader_command_received_at = time.monotonic()

    def _on_follower_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self._follower_pose = PlanarPose(
            float(position.x), float(position.y), yaw_from_odometry(message)
        )
        self._follower_received_at = time.monotonic()
        if self._leader_start is not None:
            self._append_follower_actual(self._follower_pose)
            assert self._follower_start is not None
            self._publish_relative_odometry(
                self._follower_relative_odom_pub,
                self._follower_pose,
                self._follower_start,
                "follower_start",
            )

    def _transform_leader_pose(self, pose: PlanarPose) -> PlanarPose:
        leader_start = self._leader_start
        follower_start = self._follower_start
        assert leader_start is not None and follower_start is not None

        dx = pose.x - leader_start.x
        dy = pose.y - leader_start.y
        c_leader = math.cos(leader_start.yaw)
        s_leader = math.sin(leader_start.yaw)
        local_x = c_leader * dx + s_leader * dy
        local_y = -s_leader * dx + c_leader * dy

        # The robots face each other. A Leader reverse displacement therefore
        # becomes a Follower forward displacement.
        if bool(self.get_parameter("opposite_facing").value):
            local_x = -local_x
            local_y = -local_y

        c_follower = math.cos(follower_start.yaw)
        s_follower = math.sin(follower_start.yaw)
        return PlanarPose(
            follower_start.x + c_follower * local_x - s_follower * local_y,
            follower_start.y + s_follower * local_x + c_follower * local_y,
            normalize_angle(
                follower_start.yaw + normalize_angle(pose.yaw - leader_start.yaw)
            ),
        )

    def _publish_relative_odometry(
        self,
        publisher,
        pose: PlanarPose,
        origin: PlanarPose,
        frame_id: str,
        rotate_180: bool = False,
    ) -> None:
        dx = pose.x - origin.x
        dy = pose.y - origin.y
        c = math.cos(origin.yaw)
        s = math.sin(origin.yaw)
        relative_x = c * dx + s * dy
        relative_y = -s * dx + c * dy
        if rotate_180:
            relative_x = -relative_x
            relative_y = -relative_y
        relative_yaw = normalize_angle(pose.yaw - origin.yaw)

        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.child_frame_id = frame_id + "_base"
        message.pose.pose.position.x = relative_x
        message.pose.pose.position.y = relative_y
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.z = math.sin(relative_yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(relative_yaw / 2.0)
        publisher.publish(message)

    def _append_leader_path(self, pose: PlanarPose) -> None:
        spacing = float(self.get_parameter("path_point_spacing").value)
        if self._path and math.hypot(
            pose.x - self._path[-1].x, pose.y - self._path[-1].y
        ) < spacing:
            return
        self._path.append(pose)
        self._trim_path(self._path)
        self._publish_path(self._leader_path_pub, self._path)

    def _append_follower_actual(self, pose: PlanarPose) -> None:
        spacing = float(self.get_parameter("path_point_spacing").value)
        if self._follower_actual and math.hypot(
            pose.x - self._follower_actual[-1].x,
            pose.y - self._follower_actual[-1].y,
        ) < spacing:
            return
        self._follower_actual.append(pose)
        self._trim_path(self._follower_actual)
        self._publish_path(self._follower_path_pub, self._follower_actual)

    def _trim_path(self, poses: List[PlanarPose]) -> None:
        maximum = int(self.get_parameter("maximum_path_points").value)
        if len(poses) > maximum:
            del poses[: len(poses) - maximum]
            self._progress_index = max(0, self._progress_index - 1)

    def _publish_path(self, publisher, poses: List[PlanarPose]) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        for pose in poses:
            stamped = PoseStamped()
            stamped.header = message.header
            stamped.pose.position.x = pose.x
            stamped.pose.position.y = pose.y
            stamped.pose.orientation.z = math.sin(pose.yaw / 2.0)
            stamped.pose.orientation.w = math.cos(pose.yaw / 2.0)
            message.poses.append(stamped)
        publisher.publish(message)

    def _read_keyboard(self) -> None:
        while select.select([self._stdin_fd], [], [], 0.0)[0]:
            data = os.read(self._stdin_fd, 32)
            if not data:
                break
            self._input_buffer += data.decode("latin1")

        while self._input_buffer:
            if self._input_buffer.startswith("\x1b"):
                if len(self._input_buffer) < 3:
                    return
                sequence = self._input_buffer[:3]
                self._input_buffer = self._input_buffer[3:]
                if not self._active and sequence in ARROW_KEYS:
                    self._set_manual_motion(ARROW_KEYS[sequence])
                continue

            key = self._input_buffer[0].lower()
            self._input_buffer = self._input_buffer[1:]
            if key == "p":
                if self._active:
                    self._switch_to_manual()
                else:
                    self._start_automatic()
            elif key == " ":
                self._switch_to_manual()
                self.get_logger().warn("STOPPED by SPACE; MANUAL mode selected.")
            elif key == "r":
                self._reset_session()
            elif not self._active and key in COMBINATION_KEYS:
                self._set_manual_motion(COMBINATION_KEYS[key])

    def _set_manual_motion(self, motion: str) -> None:
        self._manual_motion = motion
        self._last_manual_key_time = time.monotonic()

    def _poses_are_fresh(self) -> bool:
        timeout = float(self.get_parameter("pose_timeout").value)
        now = time.monotonic()
        return (
            self._leader_pose is not None
            and self._follower_pose is not None
            and now - self._leader_received_at <= timeout
            and now - self._follower_received_at <= timeout
        )

    def _start_automatic(self) -> None:
        if not self._poses_are_fresh():
            self._publish_stop()
            self.get_logger().error(
                "Cannot start: Leader/Follower odometry is missing or stale."
            )
            return
        assert self._leader_pose is not None and self._follower_pose is not None
        self._leader_start = PlanarPose(**vars(self._leader_pose))
        self._follower_start = PlanarPose(**vars(self._follower_pose))
        self._path = [PlanarPose(**vars(self._follower_start))]
        self._follower_actual = [PlanarPose(**vars(self._follower_start))]
        self._progress_index = 0
        self._manual_motion = None
        self._publish_path(self._leader_path_pub, self._path)
        self._publish_path(self._follower_path_pub, self._follower_actual)
        self._publish_relative_odometry(
            self._leader_relative_odom_pub,
            self._leader_pose,
            self._leader_start,
            "leader_start",
            rotate_180=bool(self.get_parameter("opposite_facing").value),
        )
        self._publish_relative_odometry(
            self._follower_relative_odom_pub,
            self._follower_pose,
            self._follower_start,
            "follower_start",
        )
        self._active = True
        self.get_logger().info(
            "AUTO mode: opposite-facing Leader path and speed following started."
        )

    def _switch_to_manual(self) -> None:
        self._active = False
        self._manual_motion = None
        self._leader_start = None
        self._follower_start = None
        self._publish_stop()
        self.get_logger().info("MANUAL mode: arrows/QWAS control the Follower.")

    def _reset_session(self) -> None:
        self._active = False
        self._manual_motion = None
        self._leader_start = None
        self._follower_start = None
        self._path.clear()
        self._follower_actual.clear()
        self._progress_index = 0
        self._publish_stop()
        self._publish_path(self._leader_path_pub, self._path)
        self._publish_path(self._follower_path_pub, self._follower_actual)
        self.get_logger().info("RESET complete; MANUAL mode.")

    def _manual_command(self) -> Twist:
        command = Twist()
        timeout = float(self.get_parameter("key_timeout").value)
        if (
            self._manual_motion is None
            or time.monotonic() - self._last_manual_key_time > timeout
        ):
            self._manual_motion = None
            return command

        linear = float(self.get_parameter("manual_linear_speed").value)
        angular = float(self.get_parameter("manual_angular_speed").value)
        if self._manual_motion in ("forward", "forward_left", "forward_right"):
            command.linear.x = linear
        elif self._manual_motion in ("reverse", "reverse_left", "reverse_right"):
            command.linear.x = -linear
        if self._manual_motion in ("left", "forward_left", "reverse_left"):
            command.angular.z = angular
        elif self._manual_motion in ("right", "forward_right", "reverse_right"):
            command.angular.z = -angular
        return command

    def _target_pose(self) -> Optional[PlanarPose]:
        if self._follower_pose is None or len(self._path) < 2:
            return None
        start = max(0, self._progress_index - 10)
        nearest = min(
            range(start, len(self._path)),
            key=lambda index: math.hypot(
                self._path[index].x - self._follower_pose.x,
                self._path[index].y - self._follower_pose.y,
            ),
        )
        path_error = math.hypot(
            self._path[nearest].x - self._follower_pose.x,
            self._path[nearest].y - self._follower_pose.y,
        )
        if path_error > float(self.get_parameter("max_path_error").value):
            if time.monotonic() - self._last_warning_at > 1.0:
                self.get_logger().error(
                    f"Path error {path_error:.2f} m exceeds limit; stopping."
                )
                self._last_warning_at = time.monotonic()
            return None

        self._progress_index = max(self._progress_index, nearest)
        lookahead = float(self.get_parameter("lookahead_distance").value)
        travelled = 0.0
        target_index = nearest
        for index in range(nearest + 1, len(self._path)):
            travelled += math.hypot(
                self._path[index].x - self._path[index - 1].x,
                self._path[index].y - self._path[index - 1].y,
            )
            target_index = index
            if travelled >= lookahead:
                break
        return self._path[target_index]

    def _compute_command(self, target: PlanarPose) -> Twist:
        command = Twist()
        assert self._follower_pose is not None
        command_age = time.monotonic() - self._leader_command_received_at
        command_timeout = float(
            self.get_parameter("leader_command_timeout").value
        )
        if command_age > command_timeout:
            return command

        # Opposite-facing robots use opposite signed linear velocities:
        # Leader -0.05 m/s becomes Follower +0.05 m/s and vice versa.
        max_linear = float(self.get_parameter("max_linear_speed").value)
        speed = clamp(
            -float(self._leader_command.linear.x),
            -max_linear,
            max_linear,
        )
        max_angular = float(self.get_parameter("max_angular_speed").value)
        if abs(speed) <= 1.0e-6:
            command.angular.z = clamp(
                -float(self._leader_command.angular.z),
                -max_angular,
                max_angular,
            )
            return command

        dx = target.x - self._follower_pose.x
        dy = target.y - self._follower_pose.y
        distance = math.hypot(dx, dy)
        if distance <= float(self.get_parameter("goal_tolerance").value):
            return command

        c = math.cos(self._follower_pose.yaw)
        s = math.sin(self._follower_pose.yaw)
        body_x = c * dx + s * dy
        body_y = -s * dx + c * dy
        target_angle = math.atan2(body_y, body_x)
        heading_error = target_angle
        if speed < 0.0:
            heading_error = normalize_angle(target_angle - math.pi)
        if abs(heading_error) >= float(
            self.get_parameter("turn_in_place_angle").value
        ):
            gain = float(self.get_parameter("heading_gain").value)
            command.angular.z = clamp(
                gain * heading_error, -max_angular, max_angular
            )
            return command

        command.linear.x = speed
        lookahead = max(
            float(self.get_parameter("lookahead_distance").value), 0.01
        )
        curvature = 2.0 * body_y / max(
            distance * distance, lookahead * lookahead
        )
        command.angular.z = clamp(
            float(self._leader_command.angular.z) + speed * curvature,
            -max_angular,
            max_angular,
        )
        return command

    def _publish_target(self, target: PlanarPose) -> None:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._frame_id
        marker.ns = "leader_path_follower"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target.x
        marker.pose.position.y = target.y
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.10
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self._target_pub.publish(marker)

    def _on_timer(self) -> None:
        self._read_keyboard()
        if not self._active:
            self._command_pub.publish(self._manual_command())
            return
        if not self._poses_are_fresh():
            self._active = False
            self._publish_stop()
            self.get_logger().error("Odometry timeout; follower stopped and paused.")
            return
        target = self._target_pose()
        if target is None:
            if len(self._path) >= 2:
                self._publish_stop()
                return
            command = Twist()
            command_timeout = float(
                self.get_parameter("leader_command_timeout").value
            )
            if (
                time.monotonic() - self._leader_command_received_at
                <= command_timeout
            ):
                limit = float(self.get_parameter("max_linear_speed").value)
                linear = -float(self._leader_command.linear.x)
                command.linear.x = clamp(linear, -limit, limit)
                leader_turn = float(self._leader_command.angular.z)
                if abs(command.linear.x) <= 1.0e-6:
                    leader_turn = -leader_turn
                command.angular.z = clamp(
                    leader_turn,
                    -float(self.get_parameter("max_angular_speed").value),
                    float(self.get_parameter("max_angular_speed").value),
                )
            self._command_pub.publish(command)
            return
        self._publish_target(target)
        self._command_pub.publish(self._compute_command(target))

    def _publish_stop(self) -> None:
        self._command_pub.publish(Twist())

    def restore_terminal(self) -> None:
        if self._terminal_configured:
            termios.tcsetattr(
                self._stdin_fd, termios.TCSADRAIN, self._terminal_settings
            )
            self._terminal_configured = False

    def destroy_node(self):
        self.restore_terminal()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[LeaderPathFollower] = None
    try:
        node = LeaderPathFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node._publish_stop()
                time.sleep(0.05)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(   )
