"""Display persistent survivor registry tracks in RViz."""

from math import isfinite

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rescue_robot_interfaces.msg import SurvivorTrack, SurvivorTrackArray
from visualization_msgs.msg import Marker, MarkerArray


REGISTRY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

MARKER_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

MAX_SURVIVOR_MARKER_ID = (2**31 - 2) // 2


def build_registry_marker_array(
    message,
    previous_ids,
    namespace,
    marker_scale,
    text_height,
    text_z_offset,
):
    """Build persistent markers and explicit cleanup actions."""
    result = MarkerArray()
    current_ids = set()
    invalid_count = 0
    infinite_lifetime = Duration(seconds=0.0).to_msg()

    if not message.tracks:
        deletion = Marker()
        deletion.header = message.header
        deletion.ns = namespace
        deletion.action = Marker.DELETEALL
        result.markers.append(deletion)
        return result, current_ids, invalid_count

    for track in sorted(message.tracks, key=lambda item: item.id):
        position = track.filtered_position
        if (
            track.id < 1
            or track.id > MAX_SURVIVOR_MARKER_ID
            or track.id in current_ids
            or track.status not in (
                SurvivorTrack.STATUS_CONFIRMED,
                SurvivorTrack.STATUS_LOST,
            )
            or not all(isfinite(value) for value in (
                position.x, position.y, position.z,
            ))
        ):
            invalid_count += 1
            continue

        current_ids.add(track.id)
        visible = (
            track.visible
            and track.status == SurvivorTrack.STATUS_CONFIRMED
        )
        sphere_id = 2 * track.id
        text_id = sphere_id + 1

        sphere = Marker()
        sphere.header = message.header
        sphere.ns = namespace
        sphere.id = sphere_id
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = position
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = marker_scale
        sphere.scale.y = marker_scale
        sphere.scale.z = marker_scale
        if visible:
            sphere.color.r = 1.0
            sphere.color.g = 0.35
            sphere.color.b = 0.10
            sphere.color.a = 0.95
        else:
            sphere.color.r = 0.55
            sphere.color.g = 0.55
            sphere.color.b = 0.55
            sphere.color.a = 0.45
        sphere.lifetime = infinite_lifetime
        result.markers.append(sphere)

        label = Marker()
        label.header = message.header
        label.ns = namespace
        label.id = text_id
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = position.x
        label.pose.position.y = position.y
        label.pose.position.z = position.z + text_z_offset
        label.pose.orientation.w = 1.0
        label.scale.z = text_height
        label.color.r = 1.0 if visible else 0.75
        label.color.g = 1.0 if visible else 0.75
        label.color.b = 1.0 if visible else 0.75
        label.color.a = 1.0 if visible else 0.70
        label.lifetime = infinite_lifetime
        label.text = (
            f"Survivor #{track.id}\n"
            f"X: {position.x:.2f}\n"
            f"Y: {position.y:.2f}\n"
            f"Z: {position.z:.2f}\n"
            f"{'VISIBLE' if visible else 'LAST SEEN'}"
        )
        result.markers.append(label)

    for survivor_id in sorted(previous_ids - current_ids):
        for marker_id in (2 * survivor_id, 2 * survivor_id + 1):
            deletion = Marker()
            deletion.header = message.header
            deletion.ns = namespace
            deletion.id = marker_id
            deletion.action = Marker.DELETE
            result.markers.append(deletion)

    return result, current_ids, invalid_count


class SurvivorRegistryVisualizerNode(Node):
    """Render confirmed and lost Registry snapshots independently of raw."""

    def __init__(self):
        super().__init__("survivor_registry_visualizer")
        self._input_topic = str(self.declare_parameter(
            "input_topic", "/leader/survivor/tracks"
        ).value).strip()
        self._output_topic = str(self.declare_parameter(
            "output_topic", "/leader/survivor/registry_markers"
        ).value).strip()
        self._map_frame = str(self.declare_parameter(
            "map_frame", "map"
        ).value).strip()
        self._marker_namespace = str(self.declare_parameter(
            "marker_namespace", "survivor_registry"
        ).value).strip()
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

        self._active_ids = set()
        self._publisher = self.create_publisher(
            MarkerArray, self._output_topic, MARKER_QOS
        )
        self._subscription = self.create_subscription(
            SurvivorTrackArray,
            self._input_topic,
            self._tracks_callback,
            REGISTRY_QOS,
        )
        self.get_logger().info(
            f"Visualizing {self._input_topic} on {self._output_topic} "
            "with persistent markers"
        )

    def _validate_parameters(self):
        if not all((
            self._input_topic,
            self._output_topic,
            self._map_frame,
            self._marker_namespace,
        )):
            raise ValueError("topics, map_frame, and namespace must not be empty")
        if self._input_topic == self._output_topic:
            raise ValueError("input_topic and output_topic must differ")
        for name, value in (
            ("marker_scale", self._marker_scale),
            ("text_height", self._text_height),
        ):
            if not isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not isfinite(self._text_z_offset) or self._text_z_offset < 0.0:
            raise ValueError("text_z_offset must be finite and nonnegative")

    def _tracks_callback(self, message):
        frame_id = message.header.frame_id.strip()
        if not frame_id:
            self._warn("Skipping Registry snapshot with an empty frame_id")
            return
        if frame_id != self._map_frame:
            self._warn(
                f"Skipping Registry snapshot in {frame_id}; expected "
                f"{self._map_frame}"
            )
            return
        markers, current_ids, invalid_count = build_registry_marker_array(
            message,
            self._active_ids,
            self._marker_namespace,
            self._marker_scale,
            self._text_height,
            self._text_z_offset,
        )
        if invalid_count:
            self._warn(
                f"Skipping {invalid_count} invalid Registry track(s)"
            )
        if markers.markers:
            self._publisher.publish(markers)
        self._active_ids = current_ids

    def _warn(self, message):
        self.get_logger().warning(message, throttle_duration_sec=5.0)


def main(args=None):
    """Run until ROS shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = SurvivorRegistryVisualizerNode()
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
