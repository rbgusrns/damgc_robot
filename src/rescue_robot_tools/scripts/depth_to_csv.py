#!/usr/bin/env python3
"""Save the median distance in the center of a RealSense depth image to CSV."""

import csv
import math
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DepthCsvSaver(Node):
    def __init__(self) -> None:
        super().__init__("depth_csv_saver")
        self.topic = self.declare_parameter(
            "depth_topic", "/leader/camera/depth/image_rect_raw"
        ).value
        output_path = self.declare_parameter(
            "output_path", str(Path.home() / "jisu_ws/data/depth_distance.csv")
        ).value
        self.output_path = Path(output_path).expanduser()
        self.roi_size = int(self.declare_parameter("roi_size", 20).value)
        self.interval = float(self.declare_parameter("save_interval_sec", 1.0).value)
        self.min_distance = float(self.declare_parameter("min_distance_m", 0.10).value)
        self.max_distance = float(self.declare_parameter("max_distance_m", 10.0).value)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.frame_number = 0
        self.last_saved_time = self.get_clock().now()
        self.csv_file = None
        self.csv_writer = None
        is_new_file = not self.output_path.exists() or self.output_path.stat().st_size == 0
        self.csv_file = self.output_path.open("a", newline="", encoding="utf-8-sig")
        self.csv_writer = csv.writer(self.csv_file)
        if is_new_file:
            self.csv_writer.writerow(["timestamp", "frame_number", "distance_m", "valid_pixel_count", "center_x", "center_y", "encoding"])
            self.csv_file.flush()
        self.subscription = self.create_subscription(Image, self.topic, self.depth_callback, qos_profile_sensor_data)
        self.get_logger().info(f"구독 토픽: {self.topic}")
        self.get_logger().info(f"CSV 저장 위치: {self.output_path}")

    @staticmethod
    def _to_meters(depth_roi: np.ndarray, encoding: str) -> Optional[np.ndarray]:
        if encoding.upper() in ("16UC1", "MONO16"):
            return depth_roi.astype(np.float32) * 0.001
        if encoding.upper() == "32FC1":
            return depth_roi.astype(np.float32)
        return None

    def depth_callback(self, msg: Image) -> None:
        now = self.get_clock().now()
        if (now - self.last_saved_time).nanoseconds / 1e9 < self.interval:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            self.get_logger().error(f"뎁스 영상 변환 실패: {exc}")
            return
        if image.ndim != 2:
            self.get_logger().warning(f"예상하지 못한 영상 차원: {image.shape}")
            return
        height, width = image.shape
        cx, cy = width // 2, height // 2
        half = self.roi_size // 2
        depth_m = self._to_meters(image[max(0, cy-half):cy+half, max(0, cx-half):cx+half], msg.encoding)
        if depth_m is None:
            self.get_logger().error(f"지원하지 않는 뎁스 인코딩입니다: {msg.encoding}")
            return
        valid = depth_m[np.isfinite(depth_m) & (depth_m >= self.min_distance) & (depth_m <= self.max_distance)]
        self.last_saved_time = now
        if valid.size == 0:
            self.get_logger().warning("중앙 영역에 유효한 거리값이 없습니다.")
            return
        distance = float(np.median(valid))
        if not math.isfinite(distance):
            return
        self.frame_number += 1
        self.csv_writer.writerow([datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3], self.frame_number, f"{distance:.4f}", int(valid.size), cx, cy, msg.encoding])
        self.csv_file.flush()
        self.get_logger().info(f"{self.frame_number:05d} | 거리 {distance:.3f} m | 유효 픽셀 {valid.size}")

    def destroy_node(self) -> bool:
        if self.csv_file is not None and not self.csv_file.closed:
            self.csv_file.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthCsvSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
