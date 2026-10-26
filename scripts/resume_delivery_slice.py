#!/usr/bin/env python3
"""Recover the held map once after the operator confirms initial payload overlap."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/leader/rescue_robot_bringup/scripts'))
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nvblox_msgs.msg import DistanceMapSlice
from tf2_ros import Buffer,TransformListener
from delivery_slice import remove_attached_payload


def main():
    parser=argparse.ArgumentParser();parser.add_argument('log_dir');parser.add_argument('--confirmed-box-overlap',action='store_true',required=True);args=parser.parse_args()
    rclpy.init();node=Node('confirmed_payload_map_recovery');state={}
    buffer=Buffer();listener=TransformListener(buffer,node)
    sub=node.create_subscription(DistanceMapSlice,'/mapping/navigation_slice',lambda msg:state.update(grid=msg),qos_profile_sensor_data)
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
        if 'grid' not in state:continue
        try:transform=buffer.lookup_transform('odom','base_link',rclpy.time.Time());break
        except Exception:continue
    else:raise RuntimeError('No frozen map/current pickup pose; no map changes made')
    age=(node.get_clock().now()-rclpy.time.Time.from_msg(transform.header.stamp)).nanoseconds/1e9
    if not -.5<=age<=3:raise RuntimeError('Pickup pose is stale')
    p,q=transform.transform.translation,transform.transform.rotation
    pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    original=state['grid'];updated=remove_attached_payload(original,pose,.4,.05)
    changed=sum(a!=b for a,b in zip(original.data,updated.data))
    Path(args.log_dir,'payload_overlap_recovery.json').write_text(json.dumps({'operator_confirmed_box':True,'pickup_pose':pose,'cleared_known_cells':changed,'cell_padding_m':original.resolution/math.sqrt(2)},indent=2)+'\n')
    node.destroy_subscription(sub)
    pid_file=Path(args.log_dir,'relay.pid')
    if not pid_file.exists():pid_file=Path('/tmp/damgc_staged_relay.pid')
    old=int(pid_file.read_text())
    os.kill(old,signal.SIGINT)
    pid_file.write_text(str(os.getpid()))
    pub=node.create_publisher(DistanceMapSlice,'/mapping/navigation_slice',1)
    node.create_timer(.5,lambda:pub.publish(updated));pub.publish(updated)
    node.get_logger().info(f'Confirmed pickup box overlap removed once: {changed} known cells; map stays frozen')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
