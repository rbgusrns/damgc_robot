#!/usr/bin/env python3
"""Republish saved survivor records after a map/anchor restore; hold before grasp."""
import argparse
import copy
import subprocess
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from rosidl_runtime_py.set_message import set_message_fields
from rescue_robot_interfaces.msg import SurvivorTrackArray
from nvblox_msgs.msg import DistanceMapSlice
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mission_checkpoint import load_checkpoint


def main():
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args()
    manifest,people,grid=load_checkpoint(args.directory)
    rclpy.init();node=Node('restored_survivor_mission')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    people=copy.deepcopy(people)
    for track in people['tracks']:
        track['visible']=False;track['status']=2 # Saved observations are historical, not live detections.
    tracks=SurvivorTrackArray();set_message_fields(tracks,people)
    saved_slice=DistanceMapSlice();set_message_fields(saved_slice,grid)
    pub=node.create_publisher(SurvivorTrackArray,'/leader/survivor/tracks',qos)
    slice_pub=node.create_publisher(DistanceMapSlice,'/mapping/saved_navigation_slice',qos)
    control=node.create_publisher(String,'/leader/mapping/control',10)
    state={};node.create_subscription(String,'/leader/mapping/mode',lambda m:state.update(mode=m.data),qos)
    visual=node.create_client(Trigger,'/leader/localization/hold_visual')
    deadline=time.monotonic()+30;future=None
    while time.monotonic()<deadline:
        control.publish(String(data='HOLD'));rclpy.spin_once(node,timeout_sec=.1)
        if future is None and visual.service_is_ready():future=visual.call_async(Trigger.Request())
        if state.get('mode')=='HOLD' and future is not None and future.done() and future.result().success:break
    else:raise RuntimeError('Saved mission HOLD/visual pose hold not acknowledged; do not start transport')
    def publish():
        tracks.header.stamp=node.get_clock().now().to_msg();pub.publish(tracks)
        saved_slice.header.stamp=node.get_clock().now().to_msg();slice_pub.publish(saved_slice)
    publish();node.create_timer(1.,publish)
    viz=subprocess.Popen(['ros2','run','rescue_robot_survivor','survivor_registry_visualizer_node','--ros-args','-p','map_frame:=odom'])
    node.get_logger().info(f'RESTORED: {len(tracks.tracks)} saved survivors; map HOLD; no scan/grasp/motion requested')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if viz.poll() is None:viz.terminate()
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
