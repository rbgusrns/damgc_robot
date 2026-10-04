#!/usr/bin/env python3
"""Show nvblox's height-projected slice as a 2D map without inventing free cells."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from nvblox_msgs.msg import DistanceMapSlice


def occupancy_values(data, unknown_value, threshold):
    distances = np.asarray(data, dtype=np.float32)
    valid = np.isfinite(distances) & (distances != unknown_value)
    result = np.full(distances.size, -1, dtype=np.int8)
    result[valid] = 0
    result[valid & (distances <= threshold)] = 100
    return result


class SliceMap(Node):
    def __init__(self):
        super().__init__('nvblox_slice_map')
        self.declare_parameter('occupied_distance_m', 0.05)
        self.last = 0.0
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/mapping/projected_map', qos)
        self.goal_pub = self.create_publisher(PoseStamped, '/mapping/planned_goal', qos)
        self.create_subscription(DistanceMapSlice, '/nvblox_node/static_map_slice',
                                 self.on_slice, qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', self.on_path, 1)

    def on_slice(self, msg):
        now = time.monotonic()
        if now-self.last < 0.5:
            return
        if msg.width*msg.height != len(msg.data) or msg.resolution <= 0:
            return
        self.last = now
        grid = OccupancyGrid()
        grid.header = msg.header
        grid.info.width = msg.width
        grid.info.height = msg.height
        grid.info.resolution = msg.resolution
        grid.info.origin.position.x = msg.origin.x
        grid.info.origin.position.y = msg.origin.y
        grid.info.origin.orientation.w = 1.0
        grid.data = occupancy_values(msg.data, msg.unknown_value,
                     self.get_parameter('occupied_distance_m').value).tolist()
        self.map_pub.publish(grid)

    def on_path(self, msg):
        if msg.poses:
            goal = PoseStamped()
            goal.header = msg.header
            goal.pose = msg.poses[-1].pose
            self.goal_pub.publish(goal)


def main():
    rclpy.init(); node = SliceMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
