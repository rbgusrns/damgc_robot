"""Publish conservative raw Leader approach commands from coherent base samples."""

import time
from math import isfinite, sqrt
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from leader_alignment_msgs.msg import LeaderAlignmentCommand
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32
from std_srvs.srv import SetBool

from leader_approach_control.approach_controller_logic import (
    KNOWN_MODES,
    KNOWN_STATES,
    TAG_LOST,
    BaseControlMeasurement,
    ControllerParameters,
    PlanarCommand,
    compute_approach_command,
    quaternion_yaw,
    sample_is_fresh,
)


INPUT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class ApproachControllerNode(Node):
    """Consume one atomic alignment command and publish raw Twist."""

    def __init__(self) -> None:
        super().__init__("approach_controller")
        self._declare_parameters()
        self._load_and_validate_parameters()

        self._raw_pub = self.create_publisher(Twist, "approach/cmd_vel_raw", 10)
        self._command_sub = self.create_subscription(
            LeaderAlignmentCommand,
            "alignment/command",
            self._on_command,
            INPUT_QOS,
        )
        self._detected_sub = self.create_subscription(
            Bool,
            "supply/detected",
            self._on_detected,
            INPUT_QOS,
        )
        self._tag_id_sub = self.create_subscription(
            Int32,
            "supply/tag_id",
            self._on_tag_id,
            INPUT_QOS,
        )
        self._enable_service = self.create_service(
            SetBool, "approach/enable", self._on_enable
        )

        self._detected = False
        self._selected_tag_id = -1
        self._measurement: Optional[BaseControlMeasurement] = None
        self._command_stamp_seconds: Optional[float] = None
        self._command_received_seconds: Optional[float] = None
        self._command_state: Optional[str] = None
        self._command_mode: Optional[str] = None
        self._timer = self.create_timer(
            1.0 / self._publish_rate, self._on_timer
        )

        self.get_logger().info(
            "Approach controller ready: enabled=%s, raw limits="
            "(%.3fm/s, %.3frad/s), final limits=(%.3fm/s, %.3frad/s)"
            % (
                self._enabled,
                self._control_parameters.max_raw_linear_speed,
                self._control_parameters.max_raw_angular_speed,
                self._control_parameters.max_final_linear_speed,
                self._control_parameters.max_final_angular_speed,
            )
        )

    def _declare_parameters(self) -> None:
        """Declare startup configuration for topic-level controller testing."""
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("target_tag_id", 0)
        self.declare_parameter("controller_enabled_on_startup", False)
        self.declare_parameter("controller_publish_rate", 20.0)
        self.declare_parameter("controller_pose_timeout", 0.35)

        # Provisional software-validation values, not calibrated motor tuning.
        self.declare_parameter("linear_gain", 0.20)
        self.declare_parameter("angular_gain", 0.80)
        self.declare_parameter("lateral_gain", 0.50)
        self.declare_parameter("max_raw_linear_speed", 0.05)
        self.declare_parameter("max_raw_angular_speed", 0.20)
        self.declare_parameter("near_max_angular_speed", 0.10)
        self.declare_parameter("max_final_linear_speed", 0.02)
        self.declare_parameter("max_final_angular_speed", 0.08)

    def _load_and_validate_parameters(self) -> None:
        """Load parameters and reject non-finite or unsafe configurations."""
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._target_tag_id = int(self.get_parameter("target_tag_id").value)
        self._enabled = bool(
            self.get_parameter("controller_enabled_on_startup").value
        )
        self._publish_rate = float(
            self.get_parameter("controller_publish_rate").value
        )
        self._pose_timeout = float(
            self.get_parameter("controller_pose_timeout").value
        )
        self._control_parameters = ControllerParameters(
            linear_gain=float(self.get_parameter("linear_gain").value),
            angular_gain=float(self.get_parameter("angular_gain").value),
            lateral_gain=float(self.get_parameter("lateral_gain").value),
            max_raw_linear_speed=float(
                self.get_parameter("max_raw_linear_speed").value
            ),
            max_raw_angular_speed=float(
                self.get_parameter("max_raw_angular_speed").value
            ),
            near_max_angular_speed=float(
                self.get_parameter("near_max_angular_speed").value
            ),
            max_final_linear_speed=float(
                self.get_parameter("max_final_linear_speed").value
            ),
            max_final_angular_speed=float(
                self.get_parameter("max_final_angular_speed").value
            ),
        )

        if not self._base_frame:
            raise ValueError("base_frame must not be empty")
        if self._target_tag_id < 0:
            raise ValueError("target_tag_id must be non-negative")
        timing_values = (
            self._publish_rate,
            self._pose_timeout,
        )
        if not all(isfinite(value) for value in timing_values):
            raise ValueError("Controller timing parameters must be finite")
        if self._publish_rate <= 0.0:
            raise ValueError("controller_publish_rate must be greater than zero")
        if self._pose_timeout <= 0.0:
            raise ValueError("controller_pose_timeout must be greater than zero")
        self._control_parameters.validate()

    def _on_detected(self, message: Bool) -> None:
        """Invalidate cached control data across detection transitions."""
        detected = bool(message.data)
        if detected != self._detected:
            self._invalidate_sample()
        self._detected = detected

    def _on_tag_id(self, message: Int32) -> None:
        """Invalidate cached control data whenever selected identity changes."""
        tag_id = int(message.data)
        if tag_id != self._selected_tag_id:
            self._invalidate_sample()
        self._selected_tag_id = tag_id

    def _on_command(self, message: LeaderAlignmentCommand) -> None:
        """Validate and cache one atomic pose/mode/state command."""
        received_seconds = time.monotonic()
        now_seconds = self.get_clock().now().nanoseconds / 1.0e9
        stamp_seconds = message.header.stamp.sec + message.header.stamp.nanosec / 1.0e9
        pose = message.target_pose
        if (
            not self._detected
            or self._selected_tag_id != self._target_tag_id
            or message.header.frame_id != self._base_frame
            or not sample_is_fresh(
                now_seconds,
                stamp_seconds,
                received_seconds,
                received_seconds,
                self._pose_timeout,
            )
            or message.control_mode not in KNOWN_MODES
            or message.alignment_state not in KNOWN_STATES
            or not self._pose_is_valid(pose)
        ):
            self._invalidate_sample()
            return

        x = float(pose.position.x)
        y = float(pose.position.y)
        orientation = pose.orientation
        yaw = quaternion_yaw(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )
        if yaw is None:
            self._invalidate_sample()
            return
        self._measurement = BaseControlMeasurement(
            target_x=x,
            target_y=y,
            target_yaw=yaw,
        )
        self._command_stamp_seconds = stamp_seconds
        self._command_received_seconds = received_seconds
        self._command_mode = str(message.control_mode)
        self._command_state = str(message.alignment_state)

    def _on_enable(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Enable or disable raw command generation with a fresh-sample barrier."""
        self._enabled = bool(request.data)
        self._invalidate_sample()
        response.success = True
        response.message = (
            "approach controller enabled; waiting for a fresh coherent sample"
            if self._enabled
            else "approach controller disabled"
        )
        self._raw_pub.publish(Twist())
        self.get_logger().info(response.message)
        return response

    def _on_timer(self) -> None:
        """Publish a raw command every cycle, including explicit zero commands."""
        now_seconds = self.get_clock().now().nanoseconds / 1.0e9
        received_now_seconds = time.monotonic()
        fresh = (
            self._command_stamp_seconds is not None
            and self._command_received_seconds is not None
            and sample_is_fresh(
                now_seconds,
                self._command_stamp_seconds,
                received_now_seconds,
                self._command_received_seconds,
                self._pose_timeout,
            )
        )
        command = compute_approach_command(
            self._command_state or TAG_LOST,
            self._command_mode or TAG_LOST,
            self._measurement,
            self._control_parameters,
            enabled=self._enabled,
            detected=self._detected,
            tag_valid=self._selected_tag_id == self._target_tag_id,
            fresh=fresh,
            coherent=self._measurement is not None,
        )
        self._raw_pub.publish(self._to_twist(command))

    def _invalidate_sample(self) -> None:
        """Discard every value that could otherwise reactivate an old command."""
        self._measurement = None
        self._command_stamp_seconds = None
        self._command_received_seconds = None
        self._command_state = None
        self._command_mode = None

    @staticmethod
    def _pose_is_valid(message) -> bool:
        """Validate all pose components even though control uses only x and y."""
        position = message.position
        orientation = message.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if not all(isfinite(value) for value in values):
            return False
        norm = sqrt(
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        return isfinite(norm) and norm > 1.0e-12

    @staticmethod
    def _to_twist(command: PlanarCommand) -> Twist:
        """Populate only the two axes supported by differential drive."""
        message = Twist()
        message.linear.x = command.linear_x
        message.angular.z = command.angular_z
        return message

    def stop(self) -> None:
        """Publish an explicit zero command before process shutdown."""
        self._raw_pub.publish(Twist())


def main(args: Optional[List[str]] = None) -> None:
    """Initialize ROS 2 and spin the approach controller."""
    rclpy.init(args=args)
    node: Optional[ApproachControllerNode] = None
    try:
        node = ApproachControllerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                if rclpy.ok():
                    node.stop()
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


if __name__ == "__main__":
    main()
