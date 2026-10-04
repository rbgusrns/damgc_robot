#!/usr/bin/env python3
"""Record a manually driven turn; optionally calculate (never apply) track width."""
import argparse
import json
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--duration', type=float, default=60)
parser.add_argument('--imu-topic', default='/leader/imu/data_raw')
parser.add_argument('--stop-file', help='Finish and save when this file appears')
parser.add_argument('--measured-deg', type=float, help='Independent signed physical turn angle, not the commanded angle')
parser.add_argument('--track-width', type=float, default=0.2453246)
parser.add_argument('--output', default='log/wheel_turn_' + time.strftime('%Y%m%d_%H%M%S') + '.json')
args = parser.parse_args()
if args.duration <= 0 or args.track_width <= 0 or args.measured_deg == 0:
    parser.error('duration/track width must be positive and measured angle nonzero')
from rclpy.signals import SignalHandlerOptions
import signal
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
stop_requested = False
def request_stop(signum, frame):
    global stop_requested
    stop_requested = True
signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)
node = Node('wheel_turn_measurement')
rows = {'wheel': [], 'local': [], 'imu': []}
def add_odom(message, key):
    q = message.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    rows[key].append([message.header.stamp.sec + message.header.stamp.nanosec*1e-9, yaw])
def wheel(message): add_odom(message, 'wheel')
def local(message): add_odom(message, 'local')
def imu(message):
    rows['imu'].append([message.header.stamp.sec + message.header.stamp.nanosec*1e-9, message.angular_velocity.z])
subscriptions = [node.create_subscription(Odometry, '/leader/odom/raw', wheel, qos_profile_sensor_data),
                 node.create_subscription(Odometry, '/leader/odometry/local', local, qos_profile_sensor_data),
                 node.create_subscription(Imu, args.imu_topic, imu, qos_profile_sensor_data)]
print('Recording only; no movement commands. Begin/end stationary; turn manually during capture.', flush=True)
end = time.monotonic() + args.duration
try:
    while not stop_requested and time.monotonic() < end and not (args.stop_file and Path(args.stop_file).exists()):
        rclpy.spin_once(node, timeout_sec=0.05)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
arrays = {}
for key, values in rows.items():
    if len(values) < 2:
        raise SystemExit('Insufficient samples for ' + key)
    array = np.asarray(values)
    array = array[np.argsort(array[:, 0])]
    array = array[np.r_[True, np.diff(array[:, 0]) > 0]]
    if len(array) < 2:
        raise SystemExit('Insufficient distinct timestamps for ' + key)
    arrays[key] = array
begin = max(a[0, 0] for a in arrays.values())
finish = min(a[-1, 0] for a in arrays.values())
if finish <= begin:
    raise SystemExit('No overlapping sensor interval')
report = {'imu_topic': args.imu_topic, 'common_interval_s': finish-begin, 'track_width_m': args.track_width,
          'independent_measured_deg': args.measured_deg, 'samples': {k:len(a) for k,a in arrays.items()}}
for key in ('wheel', 'local'):
    a = arrays[key]
    yaw = np.unwrap(a[:, 1])
    report[key + '_deg'] = float(np.degrees(np.interp(finish,a[:,0],yaw)-np.interp(begin,a[:,0],yaw)))
a = arrays['imu']
times = np.r_[begin, a[(a[:,0]>begin)&(a[:,0]<finish),0], finish]
report['imu_deg'] = float(np.degrees(np.trapz(np.interp(times,a[:,0],a[:,1]),times)))
report['wheel_minus_imu_deg'] = report['wheel_deg'] - report['imu_deg']
report['max_header_gap_ms'] = {k:float(np.diff(a[:,0]).max()*1000) for k,a in arrays.items()}
if args.measured_deg is not None:
    if report['wheel_deg'] * args.measured_deg <= 0:
        raise SystemExit('Measured angle and wheel rotation must have the same sign')
    report['suggested_track_width_m'] = args.track_width * report['wheel_deg'] / args.measured_deg
    report['calibration_applied'] = False
output = Path(args.output)
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(report, indent=2)+'\n')
np.savez(output.with_suffix('.npz'), **arrays)
print(json.dumps(report, indent=2))
print('Saved ' + str(output))
