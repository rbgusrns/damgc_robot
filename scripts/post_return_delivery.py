#!/usr/bin/env python3
"""Created only after the original mapping mission has returned and stopped."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer,TransformListener
from solo_delivery import SoloDelivery
from staged_delivery_runtime import wait_for_active


def matching_processes(kind):
    names = ({'planner_server','controller_server','behavior_server','bt_navigator','lifecycle_manager'}
             if kind=='nav2' else {'realsense2_camera_node','isaac_ros_visual_slam','nvblox_node'})
    found=[]
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            args=[v.decode(errors='replace') for v in path.read_bytes().split(b'\0') if v]
            if args and Path(args[0]).name in names:
                if kind=='nav2' and Path(args[0]).name=='lifecycle_manager' and not any('nvblox_nav2' in a for a in args):
                    continue
                found.append(int(path.parent.name))
        except OSError:pass
    return found


def stop_processes(pids,pump):
    for pid in pids:
        try:os.kill(pid,signal.SIGINT)
        except ProcessLookupError:pass
    deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        alive=[]
        for pid in pids:
            try:
                fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
                if fields[0]!='Z':alive.append(pid)
            except OSError:pass
        if not alive:return
        pump()
    for pid in alive:
        try:os.kill(pid,signal.SIGTERM)
        except ProcessLookupError:pass
    deadline=time.monotonic()+3
    while time.monotonic()<deadline:
        if not any(Path(f'/proc/{pid}').exists() for pid in alive):return
        pump()
    # Camera shutdown can wedge in the USB driver; terminate only the selected old phase nodes.
    for pid in alive:
        try:os.kill(pid,signal.SIGKILL)
        except ProcessLookupError:pass
    deadline=time.monotonic()+3
    while time.monotonic()<deadline:
        if not any(Path(f'/proc/{pid}').exists() for pid in alive):return
        pump()
    raise RuntimeError('Old phase processes did not stop; transport not started')


class PoseAnchor(Node):
    def __init__(self,context):
        super().__init__('delivery_pose_anchor')
        self.start_pose=tuple(context['start_pose'])
        self._pending_actions=set();self._pending_selectors=set()
        self.buffer=Buffer()
        self.listener=TransformListener(self.buffer,self,spin_thread=True,
            qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.robot_yaw=0.

    def _robot_pose(self,frame):
        transform=self.buffer.lookup_transform(frame,'base_link',Time())
        age=(self.get_clock().now()-Time.from_msg(transform.header.stamp)).nanoseconds/1e9
        if not -.5<=age<=3:raise RuntimeError('Wheel/IMU pose stale during delivery')
        p,q=transform.transform.translation,transform.transform.rotation
        self.robot_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return p.x,p.y

    def destroy_node(self):
        self.listener.executor.shutdown()
        self.listener.dedicated_listener_thread.join()
        return super().destroy_node()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('log_dir');parser.add_argument('--side-delivery',action='store_true');parser.add_argument('--resume-held',action='store_true');parser.add_argument('--transport-only',action='store_true');args=parser.parse_args()
    log_dir=Path(args.log_dir)
    context=json.loads((log_dir/'return_context.json').read_text())
    if not context.get('returned') or context['map_frame']!='odom':raise RuntimeError('No valid completed return context')
    if args.transport_only:
        grasp_log=(log_dir/'solo_grasp.log').read_text(errors='replace')
        if 'DONE gripper target sent and close wait elapsed' not in grasp_log or not (log_dir/'vision_stop_requested').exists():
            raise RuntimeError('No recorded completed grasp for transport-only resume')
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    delivery=SoloDelivery(use_existing_grasp=args.resume_held or args.transport_only);anchor=PoseAnchor(context)
    executor=SingleThreadedExecutor();executor.add_node(delivery)
    def pump():
        executor.spin_once(timeout_sec=.05)
        if delivery.error:raise RuntimeError(delivery.error)
    def prepare_grasp():
        delivery.report('GRASP_PREPARING: frozen map; switching to RGB-only 30fps')
        (log_dir/'vision_stop_requested').write_text('mapping complete; stop idle detector container\n')
        stop_processes(matching_processes('vision'),pump)
        (log_dir/'rgb_camera_requested').write_text('RGB-only 640x480 at 30fps\n')
        until=time.monotonic()+35
        while time.monotonic()<until:
            pump()
            if (log_dir/'rgb_camera_ready').exists():return
        raise RuntimeError('RGB-only camera did not become ready before grasp')

    def prepare_transport():
        delivery.report('TRANSPORT_PREPARING: stopping camera/VSLAM/3D integration after confirmed grasp')
        (log_dir/'vision_stop_requested').write_text('grasp DONE; stop idle detector container\n')
        stop_processes(matching_processes('vision'),pump)
        # Only Nav2 is reconfigured here; mapping, saved coordinates and wheel/IMU never restart.
        stop_processes(matching_processes('nav2'),pump)
        for path in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                args=[v.decode(errors='replace') for v in path.read_bytes().split(b'\0') if v]
                if len(args)>1 and args[1].endswith('/mapping_mode_manager.py'):
                    os.kill(int(path.parent.name),signal.SIGINT)
            except OSError:pass
        footprint=subprocess.Popen(['python3','/workspaces/isaac_ros-dev/scripts/delivery_footprint.py'],start_new_session=True)
        (log_dir/'footprint.pid').write_text(str(footprint.pid))
        target='/workspaces/isaac_ros-dev/src/leader/rescue_robot_bringup/launch/reverse_delivery_nav2.launch.py'
        nav_log=open(log_dir/'delivery_nav2.log','a',buffering=1)
        navigation=subprocess.Popen(['ros2','launch',target],stdout=nav_log,stderr=subprocess.STDOUT,start_new_session=True)
        (log_dir/'navigation.pid').write_text(str(navigation.pid))
        clients=[delivery.create_client(GetState,'/'+name+'/get_state') for name in ('planner_server','controller_server')]
        wait_for_active(clients,pump,GetState.Request)
        delivery.report('TRANSPORT_READY: frozen map and wheel/IMU; camera processing stopped')

    try:
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            pump()
            if delivery.people and delivery.slice is not None:
                try:anchor._robot_pose('odom');break
                except Exception:pass
        delivery.run(executor,anchor,log_dir,prepare_transport=None if args.transport_only else prepare_transport,resume_held=args.resume_held,transport_only=args.transport_only,prepare_grasp=None if args.resume_held or args.transport_only else prepare_grasp,side_delivery=args.side_delivery)
        (log_dir/'post_return_finished').write_text('See solo_delivery_result.json and post_return.log\n')
        # Retain gripper torque and the frozen map until explicit operator shutdown.
        while rclpy.ok():executor.spin_once(timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:
        delivery.moving=False;delivery.drive.publish(__import__('geometry_msgs.msg',fromlist=['Twist']).Twist())
        if delivery.active_handle is not None:delivery.active_handle.cancel_goal_async()
        executor.shutdown();delivery.destroy_node();anchor.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
