#!/usr/bin/env python3
"""Publish calibrated yaw rate while preserving the raw IMU stream."""
from copy import deepcopy
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuGyroCalibration(Node):
    def __init__(self):
        super().__init__('imu_gyro_calibration')
        self.declare_parameter('gyro_z_scale', 0.9684782203835076)
        self.declare_parameter('gyro_z_bias_rad_s', 0.0)
        self.scale = float(self.get_parameter('gyro_z_scale').value)
        self.bias = float(self.get_parameter('gyro_z_bias_rad_s').value)
        if not math.isfinite(self.scale) or self.scale <= 0 or not math.isfinite(self.bias):
            raise ValueError('Invalid gyro calibration')
        self.publisher = self.create_publisher(Imu, '/leader/imu/data_calibrated', 20)
        self.subscription = self.create_subscription(
            Imu, '/leader/imu/data_raw', self.callback, qos_profile_sensor_data)
        self.get_logger().info(f'gyro z: (raw - {self.bias}) * {self.scale}; raw topic preserved')

    def callback(self, message):
        output = deepcopy(message)
        output.angular_velocity.z = (message.angular_velocity.z - self.bias) * self.scale
        if output.angular_velocity_covariance[0] >= 0:
            for index in (2, 5, 6, 7):
                output.angular_velocity_covariance[index] *= self.scale
            output.angular_velocity_covariance[8] *= self.scale ** 2
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = ImuGyroCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
