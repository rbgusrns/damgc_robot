"""Display current survivor map positions as short-lived RViz markers."""

from math import isfinite

from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray


POSITION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def build_marker_array(
    message, previous_indices, namespace, lifetime_sec, marker_scale,
    text_height, text_z_offset,
):
    """Build current markers and explicit deletions for missing indices."""
    result = MarkerArray()
    current_indices = set()
    invalid_count = 0
    lifetime = Duration(seconds=lifetime_sec).to_msg()

    for index, pose in enumerate(message.poses):
        position = pose.position
        if not all(isfinite(value) for value in (
            position.x, position.y, position.z,
        )):
            invalid_count += 1
            continue

        # Array indices name current-frame candidates, not persistent people.
        current_indices.add(index)
        sphere = Marker()
        sphere.header = message.header
        sphere.ns = namespace
        sphere.id = 2 * index
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = position
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = marker_scale
        sphere.scale.y = marker_scale
        sphere.scale.z = marker_scale
        sphere.color.r = 1.0
        sphere.color.g = 0.35
        sphere.color.b = 0.10
        sphere.color.a = 0.95
        sphere.lifetime = lifetime
        result.markers.append(sphere)

        label = Marker()
        label.header = message.header
        label.ns = namespace
        label.id = 2 * index + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = position.x
        label.pose.position.y = position.y
        label.pose.position.z = position.z + text_z_offset
        label.pose.orientation.w = 1.0
        label.scale.z = text_height
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.lifetime = lifetime
        label.text = (
            f"Survivor candidate {index + 1}\n"
            f"X={position.x:.2f} Y={position.y:.2f} Z={position.z:.2f}"
        )
        result.markers.append(label)

    for index in sorted(previous_indices - current_indices):
        for marker_id in (2 * index, 2 * index + 1):
            deletion = Marker()
            deletion.header = message.header
            deletion.ns = namespace
            deletion.id = marker_id
            deletion.action = Marker.DELETE
            result.markers.append(deletion)

    return result, current_indices, invalid_count


class SurvivorMapVisualizerNode(Node):
    """Visualize PoseArray candidates without claiming stable identities."""

    def __init__(self):
        super().__init__("survivor_map_visualizer")
        self._input_topic = str(self.declare_parameter(
            "input_topic", "/leader/survivor/map_positions"
        ).value).strip()
        self._output_topic = str(self.declare_parameter(
            "output_topic", "/leader/survivor/map_markers"
        ).value).strip()
        self._marker_namespace = str(self.declare_parameter(
            "marker_namespace", "survivor_current"
        ).value).strip()
        self._marker_lifetime_sec = float(self.declare_parameter(
            "marker_lifetime_sec", 2.0
        ).value)
        self._marker_scale = float(self.declare_parameter(
            "marker_scale", 0.20
        ).value)
        self._text_height = float(self.declare_parameter(
            "text_height", 0.18
        ).value)
        self._text_z_offset = float(self.declare_parameter(
            "text_z_offset", 0.30
        ).value)
        self._validate_parameters()

        self._active_indices = set()
        self._publisher = self.create_publisher(
            MarkerArray, self._output_topic, POSITION_QOS
        )
        self._subscription = self.create_subscription(
            PoseArray, self._input_topic, self._positions_callback,
            POSITION_QOS,
        )
        self.get_logger().info(
            f"Visualizing {self._input_topic} on {self._output_topic} "
            f"with {self._marker_lifetime_sec:.2f}s marker lifetime"
        )

    def _validate_parameters(self):
        if not self._input_topic or not self._output_topic:
            raise ValueError("input_topic and output_topic must not be empty")
        if self._input_topic == self._output_topic:
            raise ValueError("input_topic and output_topic must differ")
        if not self._marker_namespace:
            raise ValueError("marker_namespace must not be empty")
        for name, value in (
            ("marker_lifetime_sec", self._marker_lifetime_sec),
            ("marker_scale", self._marker_scale),
            ("text_height", self._text_height),
        ):
            if not isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not isfinite(self._text_z_offset) or self._text_z_offset < 0.0:
            raise ValueError("text_z_offset must be finite and nonnegative")

    def _positions_callback(self, message):
        if not message.header.frame_id.strip():
            self.get_logger().warning(
                "Skipping map positions with an empty frame_id",
                throttle_duration_sec=5.0,
            )
            return
        markers, current_indices, invalid_count = build_marker_array(
            message,
            self._active_indices,
            self._marker_namespace,
            self._marker_lifetime_sec,
            self._marker_scale,
            self._text_height,
            self._text_z_offset,
        )
        if invalid_count:
            self.get_logger().warning(
                f"Skipping {invalid_count} non-finite map position(s)",
                throttle_duration_sec=5.0,
            )
        if markers.markers:
            self._publisher.publish(markers)
        self._active_indices = current_indices


def main(args=None):
    """Run until ROS shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = SurvivorMapVisualizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
