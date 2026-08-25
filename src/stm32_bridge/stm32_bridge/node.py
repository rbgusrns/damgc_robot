import struct
import threading
import time
from math import cos, sin

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String, UInt32

from . import protocol


class Stm32Bridge(Node):
    def __init__(self):
        super().__init__("stm32_bridge")
        # Jetson Orin 40-pin header pins 8/10 are commonly ttyTHS1 on
        # JetPack 6. Override this parameter if the carrier maps them to THS0.
        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 460800)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("imu_frame_id", "imu_link")
        # Physical constants from m4_firmware/DAMGC/ORIN_UART_PROTOCOL.md:
        # 127 mm wheel diameter, 5131 encoder ticks/revolution, and
        # 230 mm wheel contact-center separation.
        self.declare_parameter("wheel_radius_m", 0.0635)
        self.declare_parameter("wheel_separation_m", 0.23)
        self.declare_parameter("ticks_per_revolution", 5131)
        self.declare_parameter("cmd_timeout_ms", 200)
        self.declare_parameter("reconnect_period_s", 1.0)
        self._serial = None
        self._serial_lock = threading.Lock()
        self._parser = protocol.FrameParser()
        self._last_rx_time = None
        self._rx_frames = 0
        self._rx_sequence_drops = 0
        self._last_rx_seq = None
        self._serial_error_logged = False
        self._tx_seq = 0
        self._last_cmd = Twist()
        self._last_cmd_time = 0.0
        self._left_ticks = None
        self._right_ticks = None
        self._x = self._y = self._yaw = 0.0
        self._last_wheel_time = None
        self._open_serial()
        self._imu_pub = self.create_publisher(Imu, "imu/data_raw", 20)
        self._odom_pub = self.create_publisher(Odometry, "odom/raw", 20)
        self._state_pub = self.create_publisher(String, "system_state", 10)
        self._rx_count_pub = self.create_publisher(UInt32, "stm32_rx/frame_count", 10)
        self._rx_crc_pub = self.create_publisher(UInt32, "stm32_rx/crc_errors", 10)
        self._rx_drop_pub = self.create_publisher(UInt32, "stm32_rx/sequence_drops", 10)
        self.create_subscription(Twist, "cmd_vel", self._cmd_callback, 20)
        self.create_timer(0.005, self._read_serial)
        self.create_timer(0.02, self._send_velocity)
        self.create_timer(float(self.get_parameter("reconnect_period_s").value), self._reconnect)
        self.create_timer(1.0, self._publish_rx_stats)

    def _open_serial(self):
        try:
            import serial
            self._serial = serial.Serial(self.get_parameter("port").value,
                                         int(self.get_parameter("baudrate").value), timeout=0)
            self._serial_error_logged = False
            self.get_logger().info(f"Opened STM32 UART {self._serial.port}")
        except Exception as exc:
            self._serial = None
            if not self._serial_error_logged:
                self.get_logger().error(f"Cannot open STM32 UART: {exc}")
                self._serial_error_logged = True

    def _reconnect(self):
        if self._serial is None or not self._serial.is_open:
            self._open_serial()

    def _cmd_callback(self, msg):
        self._last_cmd = msg
        self._last_cmd_time = time.monotonic()

    def _send_velocity(self):
        if self._serial is None or not self._serial.is_open:
            return
        age_ms = (time.monotonic() - self._last_cmd_time) * 1000.0
        active = age_ms <= float(self.get_parameter("cmd_timeout_ms").value)
        v = self._last_cmd.linear.x if active else 0.0
        w = self._last_cmd.angular.z if active else 0.0
        separation = float(self.get_parameter("wheel_separation_m").value)
        left = round((v - w * separation / 2.0) * 1000.0)
        right = round((v + w * separation / 2.0) * 1000.0)
        flags = 1 if active else 0
        payload = protocol.CMD_VELOCITY_PAYLOAD.pack(max(-32768, min(32767, left)),
                                                       max(-32768, min(32767, right)), 200, flags)
        frame = protocol.encode_frame(protocol.CMD_VELOCITY, self._tx_seq, payload)
        self._tx_seq = (self._tx_seq + 1) & 0xFFFF
        with self._serial_lock:
            self._serial.write(frame)

    def _read_serial(self):
        if self._serial is None or not self._serial.is_open:
            return
        try:
            with self._serial_lock:
                data = self._serial.read(self._serial.in_waiting or 1)
        except Exception as exc:
            self.get_logger().error(f"STM32 UART read failed: {exc}")
            try:
                self._serial.close()
            except Exception:
                pass
            return
        for msg_type, seq, flags, payload in self._parser.feed(data):
            self._rx_frames += 1
            self._last_rx_time = time.monotonic()
            if self._last_rx_seq is not None:
                expected = (self._last_rx_seq + 1) & 0xFFFF
                if seq != expected:
                    self._rx_sequence_drops += 1
                    self.get_logger().warning(
                        f"STM32 RX sequence gap: expected {expected}, got {seq}")
            self._last_rx_seq = seq
            try:
                if msg_type == protocol.IMU_DATA:
                    self._publish_imu(protocol.unpack_imu(payload))
                elif msg_type == protocol.WHEEL_STATE:
                    self._publish_wheel(protocol.unpack_wheel(payload))
                elif msg_type == protocol.SYSTEM_STATE:
                    self._publish_system(protocol.unpack_system(payload))
            except (struct.error, ValueError) as exc:
                self.get_logger().warning(f"Invalid STM32 payload type 0x{msg_type:02x}: {exc}")

    def _publish_rx_stats(self):
        for publisher, value in (
            (self._rx_count_pub, self._rx_frames),
            (self._rx_crc_pub, self._parser.crc_errors),
            (self._rx_drop_pub, self._rx_sequence_drops),
        ):
            msg = UInt32()
            msg.data = value
            publisher.publish(msg)

    def _stamp(self, timestamp_us):
        # Until TIME_SYNC is implemented, use receipt time. The packet timestamp
        # remains available for the later clock-offset estimator.
        return self.get_clock().now().to_msg()

    def _publish_imu(self, data):
        msg = Imu()
        msg.header.stamp = self._stamp(data["timestamp_us"])
        msg.header.frame_id = self.get_parameter("imu_frame_id").value
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = data["accel"]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = data["gyro"]
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = data["quaternion"]
        # Initial conservative covariances for EKF integration. Orientation is
        # intentionally not fused yet because the physical IMU frame mounting
        # and BNO055 calibration status still need field verification.
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = 0.05 ** 2
        msg.angular_velocity_covariance[4] = 0.05 ** 2
        msg.angular_velocity_covariance[8] = 0.05 ** 2
        msg.linear_acceleration_covariance[0] = 0.2 ** 2
        msg.linear_acceleration_covariance[4] = 0.2 ** 2
        msg.linear_acceleration_covariance[8] = 0.2 ** 2
        self._imu_pub.publish(msg)

    def _publish_wheel(self, data):
        radius = float(self.get_parameter("wheel_radius_m").value)
        separation = float(self.get_parameter("wheel_separation_m").value)
        tpr = float(self.get_parameter("ticks_per_revolution").value)
        if self._left_ticks is not None and self._last_wheel_time is not None:
            dl = (data["left_ticks"] - self._left_ticks) * 2.0 * 3.141592653589793 * radius / tpr
            dr = (data["right_ticks"] - self._right_ticks) * 2.0 * 3.141592653589793 * radius / tpr
            ds = (dl + dr) / 2.0
            self._yaw += (dr - dl) / separation
            self._x += ds * __import__("math").cos(self._yaw)
            self._y += ds * __import__("math").sin(self._yaw)
        self._left_ticks, self._right_ticks = data["left_ticks"], data["right_ticks"]
        self._last_wheel_time = data["timestamp_us"]
        msg = Odometry()
        msg.header.stamp = self._stamp(data["timestamp_us"])
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.child_frame_id = self.get_parameter("child_frame_id").value
        msg.pose.pose.position.x, msg.pose.pose.position.y = self._x, self._y
        # The planar wheel odometry has no roll/pitch estimate. Publish the
        # integrated yaw as a normalized planar quaternion instead of leaving
        # Odometry.orientation at its default identity value.
        msg.pose.pose.orientation.z = sin(self._yaw / 2.0)
        msg.pose.pose.orientation.w = cos(self._yaw / 2.0)
        msg.twist.twist.linear.x = (data["left_mm_s"] + data["right_mm_s"]) / 2000.0
        msg.twist.twist.angular.z = (data["right_mm_s"] - data["left_mm_s"]) / (separation * 1000.0)
        # Initial wheel-odometry covariances for robot_localization. These are
        # deliberately conservative until repeated distance/rotation tests
        # provide measured uncertainties.
        msg.pose.covariance[0] = 0.01 ** 2
        msg.pose.covariance[7] = 0.01 ** 2
        msg.pose.covariance[35] = 0.05 ** 2
        msg.twist.covariance[0] = 0.02 ** 2
        msg.twist.covariance[35] = 0.05 ** 2
        self._odom_pub.publish(msg)

    def _publish_system(self, data):
        msg = String()
        msg.data = str(data)
        self._state_pub.publish(msg)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = Stm32Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
