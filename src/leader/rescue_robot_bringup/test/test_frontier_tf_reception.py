"""TF must keep updating while the frontier executor is busy or idle."""
from pathlib import Path
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('nvblox_msgs.msg')
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from frontier_explorer import FrontierExplorer


def test_tf_is_received_without_spinning_frontier_executor():
    rclpy.init(args=[])
    explorer = FrontierExplorer()
    source = rclpy.create_node('frontier_tf_test_source')
    publisher = source.create_publisher(TFMessage, '/tf', 10)
    stop = threading.Event()

    def broadcast():
        while not stop.is_set():
            transform = TransformStamped()
            transform.header.frame_id = 'odom'
            transform.header.stamp = source.get_clock().now().to_msg()
            transform.child_frame_id = 'base_link'
            transform.transform.rotation.w = 1.0
            publisher.publish(TFMessage(transforms=[transform]))
            stop.wait(0.03)

    worker = threading.Thread(target=broadcast)
    worker.start()
    try:
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            if explorer.tf_buffer.can_transform('odom', 'base_link', rclpy.time.Time()):
                break
            time.sleep(0.03)
        # The frontier node's executor is deliberately never spun. This exceeds
        # its three-second freshness allowance unless TF has its own receiver.
        time.sleep(3.3)
        assert explorer._robot_pose('odom') == (0.0, 0.0)
        transform = explorer.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        age = (source.get_clock().now()-rclpy.time.Time.from_msg(transform.header.stamp)).nanoseconds/1e9
        assert age < 0.5
    finally:
        stop.set()
        worker.join()
        explorer.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
