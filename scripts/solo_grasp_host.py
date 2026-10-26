#!/usr/bin/env python3
"""Launch the validated leader grasp task only after the mapping coordinator requests it."""
import os
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class SoloGraspHost(Node):
    def __init__(self):
        super().__init__('solo_grasp_host')
        self.phase = 'IDLE'
        self.process = None
        self.future = None
        self.started = False
        self.begin = 0.
        self.pub = self.create_publisher(String, '/leader/solo/grasp/status',
                                        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.stop = self.create_publisher(String, '/leader/command_selector/request', 10)
        self.start_client = self.create_client(Trigger, '/leader/grasp/start')
        self.cancel_client = self.create_client(Trigger, '/leader/grasp/cancel')
        self.create_subscription(String, '/leader/sequence/status', self.sequence, 10)
        self.create_service(Trigger, '/leader/solo/grasp/start', self.start)
        self.create_service(Trigger, '/leader/solo/grasp/cancel', self.cancel)
        self.create_timer(.1,self.tick)

    def status(self, text):
        self.phase = text.split(':',1)[0]
        self.pub.publish(String(data=text))
        self.get_logger().info(text)

    def start(self, request, response):
        if self.phase != 'IDLE' or self.start_client.service_is_ready():
            response.message = 'Another grasp task already exists or this cycle already started'
            return response
        self.begin = time.monotonic()
        self.process = subprocess.Popen(['ros2','launch','rescue_robot_bringup','leader_grasp_demo.launch.py',
            'start_camera:=false','start_robot_state_publisher:=false',
            'start_command_selector:=false','use_stm32_bridge:=false'])
        self.status('LAUNCHING: leader tag alignment and gripper; existing camera/drive reused')
        response.success = True
        response.message = 'Grasp preparation accepted'
        return response

    def cancel(self, request, response):
        if self.cancel_client.service_is_ready():self.cancel_client.call_async(Trigger.Request())
        self.stop.publish(String(data='STOP'))
        self.status('FAULT: cancelled; hold last gripper target')
        response.success = True
        return response

    def sequence(self,msg):
        if self.phase not in ('LAUNCHING','STARTING','RUNNING'):return
        if msg.data.startswith('STARTED'):self.started=True
        if msg.data.startswith(('FAULT','CANCELLED','ERROR')):
            self.cancel(None,Trigger.Response())
        elif msg.data.startswith('DONE') and self.started:
            self.status('DONE: motor ACK and close wait complete; lift disabled')

    def tick(self):
        if self.phase not in ('LAUNCHING','STARTING','RUNNING'):return
        if self.process.poll() is not None or time.monotonic()-self.begin>150:
            self.cancel(None,Trigger.Response());return
        if self.phase == 'LAUNCHING' and self.start_client.service_is_ready() and self.cancel_client.service_is_ready():
            self.future=self.start_client.call_async(Trigger.Request())
            self.status('STARTING: requesting validated grasp task')
        elif self.phase == 'STARTING' and self.future.done():
            try:
                if not self.future.result().success:raise RuntimeError(self.future.result().message)
                # DONE can race the start reply; do not overwrite an eventual terminal state.
                self.status('RUNNING: open, align, stop and grasp')
            except Exception:self.cancel(None,Trigger.Response())


def main():
    rclpy.init();node=SoloGraspHost()
    try:rclpy.spin(node)
    except KeyboardInterrupt:
        node.cancel(None,Trigger.Response())
        for _ in range(5):
            if rclpy.ok():rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':main()
