#!/usr/bin/env python3
"""Leader-side single signal for the two verified, independent grasp tasks."""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class PairedGrasp(Node):
    def __init__(self):
        super().__init__('paired_grasp')
        self.active = False
        self.phase = 'IDLE'
        self.started = {}
        self.done = {}
        self.futures = {}
        self.last_sequence = {}
        self.requested_at = 0.
        self.start_clients = {}
        self.cancel_clients = {}
        self.subscriptions_kept = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/leader/grasp_pair/status', qos)
        for role in ('leader', 'follower'):
            self.start_clients[role] = self.create_client(Trigger, f'/{role}/grasp/start')
            self.cancel_clients[role] = self.create_client(Trigger, f'/{role}/grasp/cancel')
            self.subscriptions_kept.append(self.create_subscription(
                String, f'/{role}/sequence/status',
                lambda msg, role=role: self._sequence(role, msg), 10))
        self.create_service(Trigger, '/leader/grasp_pair/start', self._start)
        self.create_service(Trigger, '/leader/grasp_pair/cancel', self._cancel)
        self.create_timer(.1, self._tick)
        self._status('IDLE: waiting for one leader start signal')

    def _status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _start(self, request, response):
        if self.active or self.phase != 'IDLE':
            response.message = 'Pair already started; restart coordinator for a new cycle'
            return response
        unavailable = [role for role in self.start_clients
                       if not self.start_clients[role].service_is_ready()
                       or not self.cancel_clients[role].service_is_ready()]
        if unavailable:
            response.message = 'No commands sent; missing start/cancel services: ' + ', '.join(unavailable)
            return response
        self.active = True
        self.phase = 'STARTING'
        self.requested_at = time.monotonic()
        self.started = dict.fromkeys(self.start_clients, False)
        self.done = dict.fromkeys(self.start_clients, False)
        self.last_sequence = {}
        self._status('STARTING: leader signal sent to both grasp tasks')
        # Two independent DDS requests, not a hard realtime simultaneous close.
        self.futures = {role: client.call_async(Trigger.Request())
                        for role, client in self.start_clients.items()}
        for role, future in self.futures.items():
            future.add_done_callback(lambda done, role=role: self._late_start_result(role, done))
        response.success = True
        response.message = 'Both start requests sent; completion requires both DONE events'
        return response

    def _late_start_result(self, role, future):
        # A delayed remote acceptance after cancellation must not restart motion.
        if self.phase != 'FAULT':
            return
        try:
            result = future.result()
            if result is not None and result.success and self.cancel_clients[role].service_is_ready():
                self.cancel_clients[role].call_async(Trigger.Request())
        except Exception:
            pass

    def _cancel_both(self, reason):
        if not self.active:
            return
        self.active = False
        self.phase = 'FAULT'
        for client in self.cancel_clients.values():
            if client.service_is_ready():
                client.call_async(Trigger.Request())
        self._status('FAULT: ' + reason + '; cancel requested on both; grippers hold targets')

    def _cancel(self, request, response):
        self._cancel_both('operator cancelled paired grasp')
        response.success = True
        response.message = 'Cancel requested; grippers retain their last target'
        return response

    def _sequence(self, role, msg):
        if not self.active:
            return
        text = msg.data
        if self.last_sequence.get(role) != text:
            self.last_sequence[role] = text
            self._status(f'{role.upper()}: {text}')
        if text.startswith(('FAULT', 'ERROR', 'CANCELLED')):
            self._cancel_both(f'{role}: {text}')
        elif text.startswith('STARTED'):
            self.started[role] = True
        elif text.startswith('DONE') and self.started.get(role, False):
            self.done[role] = True

    def _tick(self):
        if not self.active:
            return
        if time.monotonic() - self.requested_at > 135.:
            self._cancel_both('paired grasp timeout')
            return
        if self.phase == 'STARTING':
            for role, future in self.futures.items():
                if not future.done():
                    if time.monotonic()-self.requested_at > 5.:
                        self._cancel_both(f'{role} start response timeout')
                    return
                try:
                    result = future.result()
                    if result is None or not result.success:
                        self._cancel_both(f'{role} start rejected: {getattr(result, "message", "no result")}')
                        return
                except Exception as error:
                    self._cancel_both(f'{role} start failed: {error}')
                    return
            self.phase = 'RUNNING'
            self._status('RUNNING: both tasks accepted; each robot aligns to its own tag')
        if all(self.done.values()):
            self.active = False
            self.phase = 'DONE'
            self._status('DONE: both grasp tasks completed (motor ACK and close wait; no lift)')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start', action='store_true')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = PairedGrasp()
    deadline = time.monotonic()+90
    pending_start = args.start
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            if pending_start:
                ready = all(c.service_is_ready() for c in
                            (*node.start_clients.values(), *node.cancel_clients.values()))
                if ready:
                    result = node._start(Trigger.Request(), Trigger.Response())
                    node.get_logger().info(result.message)
                    pending_start = False
                elif time.monotonic()>deadline:
                    node._status('NOT_STARTED: both robots not discovered within 90 seconds')
                    break
    except KeyboardInterrupt:
        node._cancel_both('paired coordinator shutdown')
        for _ in range(5):
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
