"""Save a stopped returned mission against fake nvblox in an isolated ROS graph."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import pytest
rclpy=pytest.importorskip('rclpy')
pytest.importorskip('nvblox_msgs')
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,OccupancyGrid
from nvblox_msgs.msg import DistanceMapSlice
from nvblox_msgs.srv import FilePath
from rescue_robot_interfaces.msg import SurvivorTrackArray,SurvivorTrack
from std_msgs.msg import String

ROOT=Path(__file__).resolve().parents[4]


def test_save_after_return_commits_map_and_survivors_without_grasp(tmp_path):
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1' and os.environ.get('ROS_DOMAIN_ID')!='0'
    rclpy.init();n=Node('fake_checkpoint_hardware');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    odom=n.create_publisher(Odometry,'/leader/odometry/local',10)
    people=n.create_publisher(SurvivorTrackArray,'/leader/survivor/tracks',q)
    slices=n.create_publisher(DistanceMapSlice,'/nvblox_node/static_map_slice',10)
    mode=n.create_publisher(String,'/leader/mapping/mode',q)
    state={'held':False}
    def control(msg):
        if msg.data=='HOLD':state['held']=True;mode.publish(String(data='HOLD'))
    n.create_subscription(String,'/leader/mapping/control',control,10)
    def save(req,res):
        assert state['held'];Path(req.file_path).write_bytes(b'fake nvblox layers');res.success=True;return res
    n.create_service(FilePath,'/nvblox_node/save_map',save)
    def publish():
        m=Odometry();m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.position.x=.12;m.pose.pose.orientation.z=1.;odom.publish(m)
        p=SurvivorTrackArray();p.header.frame_id='odom';t=SurvivorTrack();t.id=4;t.status=1;t.filtered_position.x=2.;t.filtered_position.y=-1.;t.raw_position=t.filtered_position;p.tracks=[t];people.publish(p)
        g=DistanceMapSlice();g.header.frame_id='odom';g.width=2;g.height=2;g.resolution=.05;g.data=[1.]*4;slices.publish(g)
    n.create_timer(.05,publish)
    context=tmp_path/'return_context.json';context.write_text(json.dumps(dict(returned=True,map_frame='odom',start_pose=[0.,0.,0.])))
    target=tmp_path/'snapshot'
    proc=subprocess.Popen([sys.executable,str(ROOT/'scripts/mapping_snapshot.py'),'save',str(target),'--mission-context',str(context)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    try:
        deadline=time.monotonic()+20
        while proc.poll() is None and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
        if proc.poll() is None:proc.kill()
        output=proc.communicate(timeout=5)[0]
        assert proc.returncode==0,output
        sys.path.insert(0,str(ROOT/'scripts'))
        from mission_checkpoint import load_checkpoint
        manifest,tracks,grid=load_checkpoint(target)
        assert manifest['home_anchor']==dict(x=0.,y=0.,yaw_rad=0.)
        assert manifest['anchor']['x']==.12
        assert tracks['tracks'][0]['id']==4
        assert tracks['tracks'][0]['filtered_position']['y']==-1.
    finally:
        if proc.poll() is None:proc.kill();proc.wait()
        n.destroy_node();rclpy.try_shutdown()
