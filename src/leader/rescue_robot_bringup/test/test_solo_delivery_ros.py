"""Full handoff ordering and reverse short-step execution in an isolated ROS graph."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import math
import json

import pytest
rclpy=pytest.importorskip('rclpy')
pytest.importorskip('nvblox_msgs.msg')
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.action import ActionServer
from nav2_msgs.action import ComputePathToPose,FollowPath
from nav_msgs.msg import Path as NavPath
from nvblox_msgs.msg import DistanceMapSlice
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import SetParametersResult

sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from solo_delivery import SoloDelivery


@pytest.mark.parametrize('camera_ready',[False,True])
@pytest.mark.parametrize('side_delivery',[False,True])
def test_map_and_visual_hold_precede_grasp_then_reverse_to_survivor(tmp_path,side_delivery,camera_ready):
    rclpy.init()
    delivery=SoloDelivery()
    fake=Node('solo_fake_hardware')
    executor=SingleThreadedExecutor();executor.add_node(delivery);executor.add_node(fake)
    state={'position':(0.,0.),'held':False,'visual':False,'hold_mode':False,'grasp':False,'steps':0}
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    mode=fake.create_publisher(String,'/leader/mapping/mode',qos)
    source=fake.create_publisher(String,'/leader/command_selector/request',10)
    status=fake.create_publisher(String,'/leader/solo/grasp/status',qos)
    services=[]
    def hold(request,response):
        state['held']=True;response.success=True;return response
    def visual(request,response):
        state['visual']=True;response.success=True;return response
    def grasp(request,response):
        assert state['held'] and state['visual'] and state['hold_mode'] and state.get('rgb_ready')
        state['grasp']=True;response.success=True
        status.publish(String(data='DONE: fake motor ACK'))
        return response
    def freeze(request,response):
        assert state['grasp'];response.success=True;return response
    def cancel(request,response):response.success=True;return response
    def selector(request,response):
        response.results=[SetParametersResult(successful=True) for _ in request.parameters]
        return response
    for name,callback in [('hold_map',hold),('freeze_map',freeze)]:
        services.append(fake.create_service(Trigger,'/leader/delivery/'+name,callback))
    for topic,callback in [('/leader/localization/hold_visual',visual),('/leader/solo/grasp/start',grasp),
                           ('/leader/solo/grasp/cancel',cancel)]:
        services.append(fake.create_service(Trigger,topic,callback))
    services.append(fake.create_service(SetParameters,'/leader/command_selector/set_parameters',selector))
    def control(msg):
        if msg.data=='HOLD':
            state['hold_mode']=True
            mode.publish(String(data='HOLD'));source.publish(String(data='STOP'))
    subscription=fake.create_subscription(String,'/leader/mapping/control',control,10)
    def plan(handle):
        result=ComputePathToPose.Result();result.path=NavPath()
        result.path.header.frame_id='odom'
        x,y=state['position']
        result.path.poses=[delivery.pose(x,y,explorer.robot_yaw),handle.request.goal]
        handle.succeed();return result
    def follow(handle):
        assert state.get('transport_ready')
        assert state['grasp'] and state['held'] and state['visual']
        pose=handle.request.path.poses[-1].pose.position
        assert pose.x>=state['position'][0] # body yaw pi: increasing world x is reverse
        state['position']=(pose.x,pose.y);state['steps']+=1
        q=handle.request.path.poses[-1].pose.orientation
        explorer.robot_yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
        handle.succeed();return FollowPath.Result()
    planner=ActionServer(fake,ComputePathToPose,'/compute_path_to_pose',execute_callback=plan)
    follower=ActionServer(fake,FollowPath,'/follow_path',execute_callback=follow)
    explorer=NS(start_pose=(0.,0.,0.),robot_yaw=math.pi,_pending_actions=set(),_pending_selectors=set(),
                _robot_pose=lambda frame:state['position'])
    delivery.people={1:(2.,0.)}
    msg=DistanceMapSlice();msg.header.frame_id='odom';msg.width=120;msg.height=120
    msg.resolution=.05;msg.origin.x=-3.;msg.origin.y=-3.;msg.unknown_value=-1000.
    msg.data=[1.]*(120*120);delivery.slice=msg
    try:
        def prepare_grasp():
            assert state['held'] and state['visual'] and state['hold_mode'] and not state['grasp']
            if not camera_ready:raise RuntimeError('RGB camera not ready')
            state['rgb_ready']=True
        def prepare_transport():
            assert state['held'] and state['visual'] and state['grasp']
            state['transport_ready']=True
        delivery.run(executor,explorer,tmp_path,prepare_transport=prepare_transport,prepare_grasp=prepare_grasp,side_delivery=side_delivery)
        if not camera_ready:
            assert not state['grasp'] and state['steps']==0
            assert 'RGB camera not ready' in (tmp_path/'solo_delivery_result.json').read_text()
            return
        assert state['grasp'] and state['steps']>=2
        result=json.loads((tmp_path/'solo_delivery_result.json').read_text())
        if side_delivery:
            assert math.isclose(result['box_distance_to_person_m'],.55,abs_tol=.05)
            assert 'final_yaw' in result
        assert math.hypot(state['position'][0]-2,state['position'][1]) >= .55
        assert state['position'][0] >= 1.15
        assert 'error' not in (tmp_path/'solo_delivery_result.json').read_text()
    finally:
        executor.shutdown();planner.destroy();follower.destroy()
        delivery.destroy_node();fake.destroy_node();rclpy.shutdown()
