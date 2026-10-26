#!/usr/bin/env python3
"""Retry only the final home alignment in the unchanged live map, then release handoff."""
import argparse,json,math,re,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy
from rcl_interfaces.srv import SetParameters
from visualization_msgs.msg import MarkerArray
from nav2_msgs.action import Spin
from builtin_interfaces.msg import Duration
from post_return_delivery import PoseAnchor


def main():
    parser=argparse.ArgumentParser();parser.add_argument('log_dir');args=parser.parse_args();directory=Path(args.log_dir)
    rclpy.init();node=Node('finish_return_alignment');executor=SingleThreadedExecutor();executor.add_node(node)
    markers={}
    subscription=node.create_subscription(MarkerArray,'/frontier_explorer/markers',lambda msg:markers.update({m.id:m for m in msg.markers if m.ns=='exploration_area'}),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    selector=node.create_client(SetParameters,'/leader/command_selector/set_parameters')
    spin=ActionClient(node,Spin,'/spin')
    def wait(predicate,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.05)
            if predicate():return
        raise RuntimeError('Alignment prerequisite/action timed out')
    def result(future,seconds):wait(future.done,seconds);return future.result()
    def mode(value):
        wait(selector.service_is_ready,10)
        request=SetParameters.Request(parameters=[Parameter('source_mode',value=value).to_parameter_msg()])
        response=result(selector.call_async(request),8)
        if not response.results or not response.results[0].successful:raise RuntimeError('Selector refused')
    anchor=None;handle=None
    try:
        wait(lambda:0 in markers and 1 in markers,10)
        a,b=markers[1].points[:2]
        yaw=math.atan2(b.y-a.y,b.x-a.x)
        home=(markers[0].pose.position.x,markers[0].pose.position.y,yaw)
        anchor=PoseAnchor({'start_pose':home})
        wait(lambda:anchor.buffer.can_transform('odom','base_link',rclpy.time.Time()),8)
        x,y=anchor._robot_pose('odom')
        if math.hypot(x-home[0],y-home[1])>.25:raise RuntimeError('Robot not at returned home vicinity')
        target=yaw+math.pi
        delta=math.atan2(math.sin(target-anchor.robot_yaw),math.cos(target-anchor.robot_yaw))
        mode('STOP')
        if abs(delta)>.25:
            wait(spin.server_is_ready,10)
            request=Spin.Goal();request.target_yaw=delta;request.time_allowance=Duration(sec=35)
            handle=result(spin.send_goal_async(request),10)
            if not handle.accepted:raise RuntimeError('Spin rejected')
            mode('NAV2')
            node.get_logger().info(f'Retrying only final home rotation: {math.degrees(delta):.1f} deg')
            response=result(handle.get_result_async(),40)
            mode('STOP')
            if response.status!=4:raise RuntimeError('Collision-checked final spin did not succeed')
        anchor._robot_pose('odom')
        delta=math.atan2(math.sin(target-anchor.robot_yaw),math.cos(target-anchor.robot_yaw))
        if abs(delta)>.25:raise RuntimeError('Final heading still outside tolerance')
        mode('STOP')
        log=(directory/'automatic_exploration.log').read_text()
        count=len(re.findall(r'Frontier \d+ reached;',log))
        context={'start_pose':home,'map_frame':'odom','successful_goal_count':count,'returned':True,'alignment_retry':True}
        temporary=directory/'return_context.tmp';temporary.write_text(json.dumps(context));temporary.replace(directory/'return_context.json')
        print('FINAL_ALIGNMENT_COMPLETE; same map/home retained; staged handoff released',flush=True)
    except Exception:
        if handle is not None:handle.cancel_goal_async()
        mode('STOP')
        raise
    finally:
        executor.shutdown()
        if anchor is not None:anchor.destroy_node()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
