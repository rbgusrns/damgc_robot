#!/usr/bin/env python3
"""Pass through mapping slices; freeze and remove only the just-grasped payload ghost."""
import copy
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from nvblox_msgs.msg import DistanceMapSlice
from tf2_ros import Buffer, TransformListener


def remove_attached_payload(msg, pose, front, half):
    result = copy.deepcopy(msg)
    values = np.asarray(result.data).reshape(result.height, result.width).copy()
    iy, ix = np.indices(values.shape)
    x = result.origin.x+(ix+.5)*result.resolution-pose[0]
    y = result.origin.y+(iy+.5)*result.resolution-pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    bx, by = c*x+s*y, -s*x+c*y
    # Anchored once at the confirmed pickup, never cleared along the route.
    inside = (bx >= .10) & (bx <= front) & (np.abs(by) <= half)
    known = np.isfinite(values) & (values != result.unknown_value)
    values[inside & known & (values <= 0.)] = 1.
    result.data = values.ravel().tolist()
    return result


class DeliverySlice(Node):
    def __init__(self):
        super().__init__('delivery_slice')
        self.front = float(self.declare_parameter('payload_front_m', .40).value)
        self.half = float(self.declare_parameter('payload_half_width_m', .05).value)
        self.msg = None
        self.received = 0.
        self.frozen = False
        self.payload_removed = False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(DistanceMapSlice, '/mapping/navigation_slice', 1)
        self.create_subscription(DistanceMapSlice, '/nvblox_node/static_map_slice', self.on_slice,
                                 qos_profile_sensor_data)
        self.create_service(Trigger, '/leader/delivery/hold_map', self.hold)
        self.create_service(Trigger, '/leader/delivery/freeze_map', self.freeze)
        self.create_timer(.5, self.publish)

    def on_slice(self, msg):
        if not self.frozen:
            self.msg = msg
            self.received = time.monotonic()
            self.publish()

    def publish(self):
        if self.msg is not None:
            self.pub.publish(self.msg)

    def hold(self, request, response):
        if self.frozen:
            response.message = 'Navigation map already held'
            return response
        if self.msg is None or self.msg.header.frame_id != 'odom' or time.monotonic()-self.received > 3:
            response.message = 'No fresh navigation map before grasp'
            return response
        self.msg = copy.deepcopy(self.msg)
        self.frozen = True
        self.publish()
        response.success = True
        response.message = 'Navigation map held before grasp; integration HOLD must also be acknowledged'
        return response

    def freeze(self, request, response):
        if self.payload_removed:
            response.message = 'Pickup already frozen; no second clearing allowed'
            return response
        try:
            if self.msg is None or self.msg.header.frame_id != 'odom' or not self.frozen:
                raise ValueError('Navigation map must be held before pickup')
            transform = self.buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            age = (self.get_clock().now()-rclpy.time.Time.from_msg(transform.header.stamp)).nanoseconds/1e9
            if not -.5 <= age <= 3:
                raise ValueError('Pickup transform stale')
            p, q = transform.transform.translation, transform.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.msg = remove_attached_payload(self.msg, (p.x,p.y,yaw), self.front,self.half)
            self.frozen = True
            self.payload_removed = True
            self.publish()
            response.success = True
            response.message = 'Navigation map frozen; attached payload ghost removed only at pickup'
            self.get_logger().info(response.message)
        except Exception as error:
            response.message = str(error)
        return response


def main():
    rclpy.init();node=DeliverySlice()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':main()
