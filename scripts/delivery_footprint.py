#!/usr/bin/env python3
"""Own the loaded robot footprint after grasp, preserving the body and measured box extents."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Polygon,Point32
from std_msgs.msg import String
from delivery_geometry import LOADED_FOOTPRINT


def main():
    rclpy.init();node=Node('delivery_loaded_footprint')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publishers=[node.create_publisher(Polygon,topic,qos) for topic in ('/local_costmap/footprint','/global_costmap/footprint')]
    mode=node.create_publisher(String,'/leader/mapping/mode',qos)
    polygon=Polygon(points=[Point32(x=x,y=y,z=0.) for x,y in LOADED_FOOTPRINT])
    def publish():
        for publisher in publishers:publisher.publish(polygon)
        mode.publish(String(data='HOLD'))
    publish();node.create_timer(.5,publish)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
