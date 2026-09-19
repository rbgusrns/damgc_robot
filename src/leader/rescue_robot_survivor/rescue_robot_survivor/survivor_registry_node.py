"""Persistent survivor registry ROS 2 wrapper."""

from math import isfinite
import threading

from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from rescue_robot_interfaces.msg import SurvivorTrack, SurvivorTrackArray
from std_srvs.srv import Trigger

from rescue_robot_survivor.survivor_registry_core import (
    Position3D,
    RegistryConfig,
    SurvivorRegistry,
)


POSITION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

REGISTRY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def build_track_array(snapshots, frame_id, stamp):
    """Convert a core snapshot to its typed ROS message."""
    message = SurvivorTrackArray()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    for snapshot in snapshots:
        track = SurvivorTrack()
        track.id = snapshot.survivor_id
        track.raw_position.x = snapshot.raw_position.x
        track.raw_position.y = snapshot.raw_position.y
        track.raw_position.z = snapshot.raw_position.z
        track.filtered_position.x = snapshot.filtered_position.x
        track.filtered_position.y = snapshot.filtered_position.y
        track.filtered_position.z = snapshot.filtered_position.z
        track.status = int(snapshot.status)
        track.visible = snapshot.visible
        track.observation_count = snapshot.observation_count
        track.first_seen = Time(
            nanoseconds=snapshot.first_seen_ns
        ).to_msg()
        track.last_seen = Time(
            nanoseconds=snapshot.last_seen_ns
        ).to_msg()
        message.tracks.append(track)
    return message


class SurvivorRegistryNode(Node):
    """Consume current map detections and publish a persistent registry."""

    def __init__(self):
        super().__init__("survivor_registry")
        self._input_topic = str(self.declare_parameter(
            "input_topic", "/leader/survivor/map_positions"
        ).value).strip()
        self._tracks_topic = str(self.declare_parameter(
            "tracks_topic", "/leader/survivor/tracks"
        ).value).strip()
        self._reset_service_name = str(self.declare_parameter(
            "reset_service", "/leader/survivor/registry/reset"
        ).value).strip()
        self._map_frame = str(self.declare_parameter(
            "map_frame", "map"
        ).value).strip()
        self._registry_publish_hz = float(self.declare_parameter(
            "registry_publish_hz", 2.0
        ).value)
        config = RegistryConfig(
            association_radius_m=float(self.declare_parameter(
                "association_radius_m", 0.50
            ).value),
            reassociation_radius_m=float(self.declare_parameter(
                "reassociation_radius_m", 0.75
            ).value),
            confirm_hits=int(self.declare_parameter(
                "confirm_hits", 3
            ).value),
            tentative_timeout_sec=float(self.declare_parameter(
                "tentative_timeout_sec", 2.0
            ).value),
            visible_timeout_sec=float(self.declare_parameter(
                "visible_timeout_sec", 2.0
            ).value),
            position_ema_alpha=float(self.declare_parameter(
                "position_ema_alpha", 0.50
            ).value),
        )
        self._validate_ros_parameters()

        self._registry = SurvivorRegistry(config)
        self._registry_lock = threading.Lock()
        self._publisher = self.create_publisher(
            SurvivorTrackArray, self._tracks_topic, REGISTRY_QOS
        )
        self._subscription = self.create_subscription(
            PoseArray,
            self._input_topic,
            self._positions_callback,
            POSITION_QOS,
        )
        self._reset_service = self.create_service(
            Trigger, self._reset_service_name, self._reset_callback
        )
        self._timer = self.create_timer(
            1.0 / self._registry_publish_hz, self._timer_callback
        )
        self.get_logger().info(
            f"Registering {self._input_topic} on {self._tracks_topic} "
            f"in {self._map_frame}; reset: {self._reset_service_name}"
        )

    def _validate_ros_parameters(self):
        if not all((
            self._input_topic,
            self._tracks_topic,
            self._reset_service_name,
            self._map_frame,
        )):
            raise ValueError("topic, service, and map_frame must not be empty")
        if self._input_topic == self._tracks_topic:
            raise ValueError("input_topic and tracks_topic must differ")
        if (
            not isfinite(self._registry_publish_hz)
            or self._registry_publish_hz <= 0.0
        ):
            raise ValueError(
                "registry_publish_hz must be finite and positive"
            )

    def _positions_callback(self, message):
        frame_id = message.header.frame_id.strip()
        if not frame_id:
            self._warn("Skipping map positions with an empty frame_id")
            return
        if frame_id != self._map_frame:
            self._warn(
                f"Skipping map positions in {frame_id}; expected "
                f"{self._map_frame}"
            )
            return
        stamp = message.header.stamp
        if stamp.sec < 0 or stamp.nanosec >= 1_000_000_000 or (
            stamp.sec == 0 and stamp.nanosec == 0
        ):
            self._warn("Skipping map positions with an invalid timestamp")
            return
        timestamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        detections = tuple(
            Position3D(
                pose.position.x,
                pose.position.y,
                pose.position.z,
            )
            for pose in message.poses
        )
        now = self.get_clock().now()
        with self._registry_lock:
            result = self._registry.update(detections, timestamp_ns)
            if not result.accepted:
                self._warn(
                    "Skipping duplicate or out-of-order map positions"
                )
                return
            self._registry.advance_time(now.nanoseconds)
            snapshots = self._registry.snapshots()
        if result.invalid_detection_count:
            self._warn(
                "Skipping "
                f"{result.invalid_detection_count} non-finite detection(s)"
            )
        self._publisher.publish(build_track_array(
            snapshots, self._map_frame, now.to_msg()
        ))

    def _timer_callback(self):
        now = self.get_clock().now()
        with self._registry_lock:
            self._registry.advance_time(now.nanoseconds)
            snapshots = self._registry.snapshots()
        self._publisher.publish(build_track_array(
            snapshots, self._map_frame, now.to_msg()
        ))

    def _reset_callback(self, _request, response):
        now = self.get_clock().now()
        with self._registry_lock:
            self._registry.reset()
            snapshots = self._registry.snapshots()
        self._publisher.publish(build_track_array(
            snapshots, self._map_frame, now.to_msg()
        ))
        response.success = True
        response.message = "Survivor registry reset; next ID is 1"
        return response

    def _warn(self, message):
        self.get_logger().warning(message, throttle_duration_sec=5.0)


def main(args=None):
    """Run until ROS shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = SurvivorRegistryNode()
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
