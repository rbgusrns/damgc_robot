#!/usr/bin/env python3
"""Express the saved exploration start in the robot's current coordinate frame."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('capture_exploration_anchor')
    markers = {}
    def receive(msg):
        for marker in msg.markers:
            if marker.ns == 'exploration_area':
                markers[marker.id] = marker
    sub = node.create_subscription(MarkerArray, '/frontier_explorer/markers', receive,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    deadline = time.monotonic() + 15
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if 0 not in markers or 1 not in markers:
                continue
            try:
                tf = buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            except Exception:
                continue
            age = (node.get_clock().now().nanoseconds-tf.header.stamp.sec*10**9-tf.header.stamp.nanosec)/1e9
            if not -0.5 <= age <= 3:
                continue
            boundary = markers[1]
            if boundary.header.frame_id != 'odom' or len(boundary.points) < 2:
                raise RuntimeError('Saved area is not an odom-frame rectangle')
            a, b = boundary.points[:2]
            saved_yaw = math.atan2(b.y-a.y, b.x-a.x)
            q = tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            start = markers[0].pose.position
            dx = start.x-tf.transform.translation.x
            dy = start.y-tf.transform.translation.y
            pose = [math.cos(yaw)*dx+math.sin(yaw)*dy,
                    -math.sin(yaw)*dx+math.cos(yaw)*dy,
                    math.atan2(math.sin(saved_yaw-yaw), math.cos(saved_yaw-yaw))]
            Path(args.output).write_text(json.dumps(pose)+'\n')
            print('Saved original area/start for fresh retry:', pose)
            return
        raise RuntimeError('Could not capture a current pose and the saved exploration area')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
