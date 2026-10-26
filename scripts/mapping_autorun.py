#!/usr/bin/env python3
"""Verify live map inputs, scan, then run the bounded explorer and return home."""
import argparse
import json
import math
import os
from collections import defaultdict
from pathlib import Path
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/leader/rescue_robot_bringup/scripts'))
from frontier_explorer import FrontierExplorer
from mapping_recovery import VideoRecovery


class LiveInputs(Node):
    def __init__(self, image_timeout_sec=10.0, require_visual_odometry=True):
        super().__init__('mapping_autorun')
        self.image_timeout_sec = image_timeout_sec
        self.required_streams = ('depth', 'filtered_depth', 'local_odometry', 'map')
        if require_visual_odometry:
            self.required_streams += ('visual_odometry',)
        self.latest = {}
        self.counts = defaultdict(int)
        self.known_cells = 0
        self.mission_status = ''
        self.stop_pub = self.create_publisher(String, '/leader/command_selector/request', 10)
        self.subscriptions_kept = []
        for key, typ, topic in [
            ('depth', Image, '/leader/camera/depth/image_rect_raw'),
            ('filtered_depth', Image, '/leader/camera/depth/self_filtered'),
            ('visual_odometry', Odometry, '/visual_slam/tracking/odometry'),
            ('local_odometry', Odometry, '/leader/odometry/local'),
        ]:
            self.subscriptions_kept.append(self.create_subscription(
                typ, topic, lambda msg, key=key: self.record(key, msg), qos_profile_sensor_data))
        self.subscriptions_kept.append(self.create_subscription(
            OccupancyGrid, '/mapping/projected_map', self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        self.subscriptions_kept.append(self.create_subscription(
            String, '/frontier_explorer/status', self.on_status, 10))
        self.scan = self.create_client(Trigger, '/leader/initial_map_scan/start')

    def record(self, key, msg):
        stamp = msg.header.stamp
        previous = self.latest.get(key)
        if previous is None or (stamp.sec, stamp.nanosec) > (previous[1].sec, previous[1].nanosec):
            self.counts[key] += 1
        self.latest[key] = (time.monotonic(), stamp)

    def on_map(self, msg):
        self.record('map', msg)
        self.known_cells = sum(value >= 0 for value in msg.data)

    def on_status(self, msg):
        self.mission_status = msg.data

    def problems(self, explorer, image_age_limit=None):
        problems = []
        if explorer.checkpoint_detection and not explorer.detection_client.service_is_ready():
            problems.append('checkpoint detector: waiting for model/service')
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds
        for key in self.required_streams:
            if key not in self.latest or self.counts[key] < 3:
                problems.append(f'{key}: waiting for frames')
                continue
            received, stamp = self.latest[key]
            age = (ros_now - stamp.sec * 10**9 - stamp.nanosec) / 1e9
            image_limit = self.image_timeout_sec if image_age_limit is None else image_age_limit
            limit = image_limit if key in ('depth', 'filtered_depth') else 5.0
            if now-received > limit or not -0.5 <= age <= limit:
                problems.append(f'{key}: age={age:.2f}s, received={now-received:.2f}s ago')
        if self.known_cells == 0:
            problems.append('map: no observed cells')
        try:
            explorer._robot_pose('odom')
        except Exception as error:
            problems.append(f'robot transform: {error}')
        if explorer.start_pose is None:
            problems.append('exploration start pose not fixed')
        return problems

    def video_outage(self):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds
        for key in ('depth', 'filtered_depth'):
            if key not in self.latest:
                return f'{key} unavailable: no frames received'
            received, stamp = self.latest[key]
            age = (ros_now-stamp.sec*10**9-stamp.nanosec)/1e9
            if now-received > self.image_timeout_sec or age > self.image_timeout_sec:
                return f'{key} unavailable for {max(now-received, age):.1f}s'
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mapping-log', required=True)
    parser.add_argument('--image-timeout-sec', type=float, default=10.0)
    args = parser.parse_args()
    mapping_log = Path(args.mapping_log)
    rclpy.init(args=['--ros-args', '-p', 'dry_run:=true',
                    '-p', 'minimum_clearance_m:=0.45',
                    '-p', 'minimum_goal_distance_m:=0.45',
                    '-p', 'minimum_cluster_cells:=3',
                    '-p', 'minimum_unknown_area_m2:=0.05',
                    '-p', 'goal_timeout_sec:=120.0',
                    '-p', 'checkpoint_detection:=' + ('true' if os.environ.get('MAPPING_CHECKPOINT_DETECTION') == '1' else 'false')],
                signal_handler_options=SignalHandlerOptions.NO)
    if not 0 < args.image_timeout_sec < 300:
        raise ValueError('Image timeout must be positive and below 300 seconds')
    monitor = LiveInputs(args.image_timeout_sec,
                         require_visual_odometry=os.environ.get('MAPPING_SOLO_DELIVERY') != '1')
    explorer = FrontierExplorer()
    delivery = None
    if os.environ.get('MAPPING_SOLO_DELIVERY') == '1':
        from solo_delivery import SoloDelivery
        delivery = SoloDelivery()
    saved_pose_file = os.environ.get('MAPPING_START_POSE_FILE', '')
    if saved_pose_file:
        saved_pose = json.loads(Path(saved_pose_file).read_text())
        if len(saved_pose) != 3 or not all(math.isfinite(value) for value in saved_pose):
            raise ValueError('Invalid saved exploration start')
        explorer.start_pose = tuple(saved_pose)
        explorer.area_center = tuple(saved_pose[:2])
        explorer.area_yaw = saved_pose[2]
        explorer.map_frame = 'odom'
        explorer.visited_positions.append(tuple(saved_pose[:2]))
        monitor.get_logger().info(f'Original exploration area/start preserved across retry: {saved_pose}')
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(explorer)
    if delivery is not None:
        executor.add_node(delivery)

    def spin(seconds=0.1):
        executor.spin_once(timeout_sec=seconds)

    def wait_healthy(timeout=90):
        deadline = time.monotonic() + timeout
        stable_since = None
        last_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            spin()
            problems = monitor.problems(explorer)
            now = time.monotonic()
            if problems:
                stable_since = None
            elif stable_since is None:
                stable_since = now
            elif now-stable_since >= 2:
                monitor.get_logger().info('LIVE INPUTS READY: depth, filtered depth, pose and 2D map are current')
                return
            if now-last_log >= 5:
                monitor.get_logger().info('PREPARING: ' + ('; '.join(problems) or 'confirming stable inputs'))
                last_log = now
        raise RuntimeError('Live inputs did not remain fresh; robot motion was not started')

    try:
        wait_healthy()
        if not monitor.scan.wait_for_service(timeout_sec=5):
            raise RuntimeError('Initial scan service is unavailable')
        future = monitor.scan.call_async(Trigger.Request())
        deadline = time.monotonic() + 10
        while not future.done() and time.monotonic() < deadline:
            spin()
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError('Initial scan request failed')
        monitor.get_logger().info('SCANNING: exploration remains in preview until the full scan succeeds')
        deadline = time.monotonic() + 90
        while rclpy.ok() and time.monotonic() < deadline:
            spin()
            log = mapping_log.read_text(errors='replace')
            if 'Initial 360-degree map scan completed.' in log:
                break
            if any(text in log for text in (
                'Initial map scan ended with Nav2 status',
                'Timed out waiting for the initial scan prerequisites.',
                'Nav2 rejected the initial scan goal.',
                'Initial scan result failed:', 'Could not start the initial scan:',
                'Initial scan requires selector STOP;',
            )):
                raise RuntimeError('Initial scan failed; exploration will not start')
        else:
            raise RuntimeError('Initial scan did not complete within 90 seconds')
        wait_healthy(timeout=30)
        explorer.set_parameters([Parameter('dry_run', value=False)])
        explorer.dry_run = False
        monitor.get_logger().info('EXPLORING: 3m forward × 3m right; return to the saved start pose on completion')
        recovery = VideoRecovery()
        last_status = ''
        while rclpy.ok():
            spin()
            recovery.poll(monitor, explorer, time.monotonic())
            if monitor.mission_status != last_status:
                last_status = monitor.mission_status
                monitor.get_logger().info(last_status)
            if (explorer.stopped and not explorer.paused
                    and last_status.startswith(('COMPLETE:', 'STOPPED:'))):
                if explorer.completed_return and os.environ.get('MAPPING_POST_RETURN') == '1':
                    context = {'start_pose': list(explorer.start_pose), 'map_frame': explorer.map_frame,
                               'successful_goal_count': explorer.successful_goal_count,
                               'returned': True}
                    context_path = mapping_log.parent / 'return_context.json'
                    temporary = context_path.with_suffix('.tmp')
                    temporary.write_text(json.dumps(context))
                    temporary.replace(context_path)
                if delivery is not None and explorer.completed_return:
                    delivery.run(executor, explorer, mapping_log.parent)
                else:
                    monitor.stop_pub.publish(String(data='STOP'))
                monitor.get_logger().info('MISSION HOLD: map and RViz remain available; Ctrl-C shuts down')
                while rclpy.ok():
                    spin()
    except RuntimeError as error:
        explorer._stop(str(error))
        monitor.stop_pub.publish(String(data='STOP'))
        monitor.get_logger().error(f'MISSION STOPPED: {error}. Map remains open for inspection.')
        while rclpy.ok():
            spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            explorer._stop('Automatic mission shutting down')
            monitor.stop_pub.publish(String(data='STOP'))
            for _ in range(3):
                spin()
        if delivery is not None:
            executor.remove_node(delivery)
            delivery.destroy_node()
        executor.remove_node(explorer)
        executor.remove_node(monitor)
        explorer.destroy_node()
        monitor.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
