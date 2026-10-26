"""Sequential solo grasp and reverse delivery in the same live mapping frame."""
import copy
import math
import time
from pathlib import Path
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as NavPath
from nav2_msgs.action import ComputePathToPose, FollowPath
from nvblox_msgs.msg import DistanceMapSlice
from rescue_robot_interfaces.msg import SurvivorTrackArray
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger
from delivery_geometry import delivery_goal, delivery_side_goals, reverse_prefix, footprint_known_clear, reverse_command, LOADED_FOOTPRINT


class SoloDelivery(Node):
    def __init__(self,use_existing_grasp=False):
        super().__init__('solo_delivery')
        self.people = {}
        self.mode = ''
        self.grasp_status = ''
        self.slice = None
        self.moving = False
        self.error = None
        self.active_handle = None
        self.delivery_started = False
        self.stop_requested = False
        self.ignore_hold_stop = 0
        self.create_subscription(SurvivorTrackArray,'/leader/survivor/tracks',self.tracks,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.raw_grasp_started=False
        if use_existing_grasp:
            self.create_subscription(String,'/leader/sequence/status',self.raw_grasp_state,10)
        else:
            self.create_subscription(String,'/leader/solo/grasp/status',self.grasp_state,
                                     QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String,'/leader/mapping/mode',self.mapping_mode,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String,'/leader/command_selector/request',self.operator,10)
        self.create_subscription(DistanceMapSlice,'/mapping/navigation_slice',self.on_slice,qos_profile_sensor_data)
        self.create_subscription(Twist,'/nav2/cmd_vel',self.command,10)
        self.create_subscription(String,'/leader/dynamixel/status',self.motor_status,10)
        self.drive = self.create_publisher(Twist,'/leader/teleop/cmd_vel',10)
        self.control = self.create_publisher(String,'/leader/mapping/control',10)
        self.status = self.create_publisher(String,'/leader/solo_delivery/status',
                                            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.target_pub = self.create_publisher(PoseStamped,'/mapping/planned_goal',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.path_pub = self.create_publisher(NavPath,'/solo_delivery/reverse_path',10)
        grasp_prefix='/leader/grasp' if use_existing_grasp else '/leader/solo/grasp'
        self.grasp = self.create_client(Trigger,grasp_prefix+'/start')
        self.cancel_grasp = self.create_client(Trigger,grasp_prefix+'/cancel')
        self.hold_visual = self.create_client(Trigger,'/leader/localization/hold_visual')
        self.hold_map = self.create_client(Trigger,'/leader/delivery/hold_map')
        self.freeze = self.create_client(Trigger,'/leader/delivery/freeze_map')
        self.selector = self.create_client(SetParameters,'/leader/command_selector/set_parameters')
        self.planner = ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.follower = ActionClient(self,FollowPath,'/follow_path')

    def tracks(self,msg):
        if msg.header.frame_id == 'odom':
            for track in msg.tracks:
                p=track.filtered_position
                if all(math.isfinite(v) for v in (p.x,p.y)):
                    self.people[track.id]=(p.x,p.y)

    def grasp_state(self,msg):self.grasp_status=msg.data
    def raw_grasp_state(self,msg):
        if msg.data.startswith('STARTED'):
            self.raw_grasp_started=True;self.grasp_status='RUNNING'
        elif self.raw_grasp_started and msg.data.startswith('DONE'):
            self.grasp_status='DONE: '+msg.data
        elif self.raw_grasp_started and msg.data.startswith(('FAULT','CANCELLED','ERROR')):
            self.grasp_status='FAULT: '+msg.data

    def mapping_mode(self,msg):self.mode=msg.data
    def on_slice(self,msg):self.slice=msg
    def operator(self,msg):
        if self.delivery_started and msg.data.upper() == 'STOP':
            if self.ignore_hold_stop:
                self.ignore_hold_stop -= 1
                return
            self.stop_requested = True
            self.error = 'Operator STOP during solo handoff/delivery'
            self.drive.publish(Twist())

    def motor_status(self,msg):
        if self.delivery_started and msg.data.startswith('ERROR'):
            self.error='Gripper motor fault: '+msg.data
            self.drive.publish(Twist())

    def command(self,msg):
        if not self.moving:return
        try:
            linear,angular=reverse_command(msg.linear.x,msg.angular.z)
            command=Twist();command.linear.x=linear;command.angular.z=angular
            self.drive.publish(command)
        except ValueError as error:
            self.error=str(error);self.drive.publish(Twist())

    def report(self,text):
        self.get_logger().info(text)
        self.status.publish(String(data=text))

    def pose(self,x,y,yaw):
        p=PoseStamped();p.header.frame_id='odom';p.header.stamp=self.get_clock().now().to_msg()
        p.pose.position.x=x;p.pose.position.y=y
        p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
        return p

    def run(self,executor,explorer,log_dir,prepare_transport=None,resume_held=False,transport_only=False,prepare_grasp=None,side_delivery=False):
        def spin():
            executor.spin_once(timeout_sec=.05)
            if self.error:raise RuntimeError(self.error)
            if self.moving:explorer._robot_pose('odom')

        def wait_until(predicate,seconds,reason):
            deadline=time.monotonic()+seconds
            while rclpy.ok() and time.monotonic()<deadline:
                spin()
                if predicate():return
            raise RuntimeError(reason)

        def wait_future(future,seconds,reason):
            wait_until(future.done,seconds,reason)
            return future.result()

        def trigger(client,timeout=8):
            wait_until(client.service_is_ready,20,'Service unavailable: '+client.srv_name)
            result=wait_future(client.call_async(Trigger.Request()),timeout,'Service timed out: '+client.srv_name)
            if result is None or not result.success:
                raise RuntimeError(getattr(result,'message','Empty service response'))
            return result

        def select(source):
            wait_until(self.selector.service_is_ready,10,'Selector service unavailable')
            request=SetParameters.Request(parameters=[Parameter('source_mode',value=source).to_parameter_msg(),
                                                      Parameter('enable_nav2_goal_selection',value=False).to_parameter_msg()])
            result=wait_future(self.selector.call_async(request),8,'Selector timed out')
            if not result.results or not all(r.successful for r in result.results):
                raise RuntimeError('Selector refused delivery ownership')

        def action(client,goal,timeout,moving=False):
            wait_until(client.server_is_ready,10,'Navigation action server unavailable')
            future=client.send_goal_async(goal)
            # If acceptance arrives after a timeout/STOP, cancel rather than leave an orphan action.
            abandoned=[False]
            def accepted_late(completed):
                if abandoned[0]:
                    try:
                        handle=completed.result()
                        if handle.accepted:handle.cancel_goal_async()
                    except Exception:pass
            future.add_done_callback(accepted_late)
            try:
                handle=wait_future(future,10,'Navigation acceptance timed out')
            except BaseException:
                abandoned[0]=True
                raise
            if not handle.accepted:raise RuntimeError('Navigation action rejected')
            self.active_handle=handle
            try:
                if moving:
                    select('TELEOP')
                    self.moving=True
                result=wait_future(handle.get_result_async(),timeout,'Navigation action timed out')
                if result.status != 4:raise RuntimeError(f'Navigation action failed: {result.status}')
                return result.result
            finally:
                self.moving=False;self.drive.publish(Twist())
                handle.cancel_goal_async()
                self.active_handle=None
                if moving:select('STOP')

        if not self.people:
            self.report('NO_SURVIVOR: no confirmed person in this map; grasp/delivery skipped')
            Path(log_dir,'solo_delivery_result.json').write_text(json.dumps({'status':'NO_SURVIVOR','grasp_skipped':True})+'\n')
            return
        home=explorer.start_pose
        person_id,person=min(self.people.items(),key=lambda item:math.hypot(item[1][0]-home[0],item[1][1]-home[1]))
        result={'person_id':person_id,'person_odom_xy':person,'home':home}
        try:
            wait_until(lambda:not explorer._pending_actions and not explorer._pending_selectors,
                       15,'Home actions did not drain before grasp')
            select('STOP')
            self.delivery_started=True
            if transport_only:
                if self.mode!='HOLD':raise RuntimeError('Existing carrying HOLD required for transport-only resume')
                self.grasp_status='DONE: retained completed grasp; no gripper command sent'
                self.report('TRANSPORT_ONLY: keeping confirmed grasp and frozen map; no reopening')
            else:
                # Freeze the navigation view and disable depth integration at grasp-mode entry.
                if not resume_held:
                    trigger(self.hold_map)
                trigger(self.hold_visual)
                if not resume_held:
                    self.ignore_hold_stop=1
                    self.control.publish(String(data='HOLD'))
                wait_until(lambda:self.mode=='HOLD',8,'Carrying HOLD was not acknowledged before grasp')
                select('STOP')
                self.report(f'GRASPING_MAP_HELD: map updates off; target survivor {person_id} at {person}')
                if prepare_grasp is not None:prepare_grasp()
                trigger(self.grasp)
                wait_until(lambda:self.grasp_status.startswith(('DONE','FAULT')),160,'Grasp completion timed out')
                if not self.grasp_status.startswith('DONE'):raise RuntimeError(self.grasp_status)
                self.report('GRASPED: map remains held; removing attached payload ghost at pickup only')
                trigger(self.freeze)
                select('STOP')
                if prepare_transport is not None:
                    prepare_transport()
            wait_until(lambda:self.slice is not None,5,'No frozen navigation slice')
            if self.count_publishers('/leader/teleop/cmd_vel') != 1:
                raise RuntimeError('Another teleop publisher exists')
            rx,ry=explorer._robot_pose('odom')
            targets=delivery_side_goals(home,person,yaw_offsets=(0.,math.pi/4,-math.pi/4,math.pi/2,-math.pi/2,math.pi)) if side_delivery else [delivery_goal((rx,ry),person)]
            targets.sort(key=lambda goal:math.hypot(goal[0]-rx,goal[1]-ry) if goal else 0.)
            target=targets[0]
            if target is None:
                self.report('DELIVERED: already within 0.7m survivor stand-off');return
            result['delivery_goal']=target
            self.target_pub.publish(self.pose(*target))
            # Wait for loaded-footprint costmaps to update after HOLD.
            entered=time.monotonic()
            wait_until(lambda:time.monotonic()-entered>=2.,4,'Costmap settling timed out')
            for step in range(24):
                rx,ry=explorer._robot_pose('odom')
                if math.hypot(rx-target[0],ry-target[1]) <= (.10 if side_delivery else .15):
                    if side_delivery and abs(math.atan2(math.sin(explorer.robot_yaw-target[2]),math.cos(explorer.robot_yaw-target[2])))>.25:
                        raise RuntimeError('Side arrival heading not aligned; stopped with gripper held')
                    result['final_odom_xy']=[rx,ry]
                    result['distance_to_person_m']=math.hypot(rx-person[0],ry-person[1])
                    if side_delivery:
                        box=(rx+.35*math.cos(explorer.robot_yaw),ry+.35*math.sin(explorer.robot_yaw))
                        result['box_center_odom_xy']=box
                        result['box_distance_to_person_m']=math.hypot(box[0]-person[0],box[1]-person[1])
                        result['final_yaw']=explorer.robot_yaw
                    self.report('DELIVERED: '+('box beside recorded survivor; ' if side_delivery else 'stopped near recorded survivor; ')+'gripper holds; no release/lift')
                    return
                last_error='No admissible reverse path'
                if side_delivery:
                    candidates=[(goal[2]+(math.pi if virtual else 0),virtual,goal) for goal in targets for virtual in (False,True)]
                else:
                    candidates=[(yaw,False,target) for yaw in (target[2],explorer.robot_yaw,target[2]+.3,target[2]-.3)]
                    candidates.append((target[2]+math.pi,True,target))
                for yaw,virtual_forward,candidate_target in candidates:
                    try:
                        if side_delivery:
                            msg=self.slice
                            endpoint_grid=dict(width=msg.width,height=msg.height,resolution=msg.resolution,origin=(msg.origin.x,msg.origin.y),data=msg.data,unknown=msg.unknown_value)
                            if not footprint_known_clear(endpoint_grid,[candidate_target],polygon=LOADED_FOOTPRINT):
                                raise RuntimeError('Side-delivery pose is occupied or unknown')
                        request=ComputePathToPose.Goal();request.goal=self.pose(candidate_target[0],candidate_target[1],yaw)
                        request.planner_id='GridBased';request.use_start=virtual_forward
                        if virtual_forward:
                            request.start=self.pose(rx,ry,explorer.robot_yaw+math.pi)
                        planned=action(self.planner,request,15).path
                        if planned.header.frame_id != 'odom':raise RuntimeError('Delivery path frame mismatch')
                        poses=[]
                        for item in planned.poses:
                            p,q=item.pose.position,item.pose.orientation
                            poses.append((p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
                        if virtual_forward:
                            poses=[(x,y,math.atan2(math.sin(yaw+math.pi),math.cos(yaw+math.pi))) for x,y,yaw in poses]
                        if poses:
                            # The planner snaps its first pose to a grid centre; execution starts at actual odometry.
                            poses[0]=(rx,ry,explorer.robot_yaw)
                        if side_delivery and (not poses or math.hypot(poses[-1][0]-candidate_target[0],poses[-1][1]-candidate_target[1])>.08):
                            raise RuntimeError('Planner endpoint does not reach side-delivery pose')
                        prefix=reverse_prefix(poses)
                        msg=self.slice
                        grid={'width':msg.width,'height':msg.height,'resolution':msg.resolution,
                              'origin':(msg.origin.x,msg.origin.y),'data':msg.data,'unknown':msg.unknown_value}
                        if not footprint_known_clear(grid,poses,polygon=LOADED_FOOTPRINT):
                            raise RuntimeError('Loaded reverse path enters unknown or occupied cells')
                        target=candidate_target
                        targets=[target]
                        result['delivery_goal']=target
                        self.target_pub.publish(self.pose(*target))
                        break
                    except (RuntimeError,ValueError) as error:
                        if self.error:raise
                        last_error=str(error)
                else:
                    raise RuntimeError(last_error)
                path=NavPath();path.header=planned.header
                path.poses=[self.pose(*p) for p in prefix]
                self.path_pub.publish(path)
                follow=FollowPath.Goal();follow.path=path
                follow.controller_id='FollowPath';follow.goal_checker_id=('general_goal_checker' if side_delivery and math.hypot(prefix[-1][0]-target[0],prefix[-1][1]-target[1])<.03 else 'position_goal_checker')
                self.report(f'DELIVERING_REVERSE: step {step+1}, maximum 0.60m, speed <=0.03m/s')
                action(self.follower,follow,60,moving=True)
            raise RuntimeError('Delivery exceeded short-step limit')
        except Exception as error:
            result['error']=str(error)
            self.report('DELIVERY_STOPPED: '+str(error))
        finally:
            self.delivery_started=False
            self.moving=False;self.drive.publish(Twist())
            if self.active_handle is not None:self.active_handle.cancel_goal_async()
            # Do not open or torque-disable the gripper at completion/failure.
            if not self.grasp_status.startswith('DONE') and self.cancel_grasp.service_is_ready():
                self.cancel_grasp.call_async(Trigger.Request())
            try:select('STOP')
            except Exception:pass
            Path(log_dir,'solo_delivery_result.json').write_text(json.dumps(result,indent=2)+'\n')
