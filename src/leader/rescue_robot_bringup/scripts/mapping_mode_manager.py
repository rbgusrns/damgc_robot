#!/usr/bin/env python3
"""Manual carrying mode and return to a saved, matching map anchor."""
import json
import math
import os
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Polygon, Point32
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from nvblox_msgs.srv import FilePath

BASE_FOOTPRINT = [(-.06,-.145),(.06,-.145),(.3175,-.07),(.3175,.07),(.06,.145),(-.06,.145)]

class MappingModeManager(Node):
    def __init__(self):
        super().__init__('mapping_mode_manager')
        self.declare_parameter('anchor_directory', os.environ.get('DAMGC_MAPPING_SNAPSHOT') or '/workspaces/isaac_ros-dev/data/maps/latest')
        self.declare_parameter('mapping_session_id', os.environ.get('DAMGC_MAPPING_RUN_ID',''))
        self.declare_parameter('payload_front_m', 0.0)
        self.declare_parameter('payload_half_width_m', 0.0)
        self.selection_value = None
        self.selection_future = None
        self.selector_parameters = self.create_client(SetParameters, '/leader/command_selector/set_parameters')
        self.loading = False
        self.cooperation = False
        self.coop_pending = False
        self.coop_pub = self.create_publisher(String, "/cooperation/transport/control", 10)
        self.load_client = self.create_client(FilePath, '/nvblox_node/load_map')
        self.mode = 'MAPPING'
        self.pending = False
        self.odom_received = None
        self.odom = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/leader/mapping/mode', qos)
        self.status_pub = self.create_publisher(String, '/leader/mapping/status', qos)
        self.source_pub = self.create_publisher(String, '/leader/command_selector/request', 10)
        self.footprints = [self.create_publisher(Polygon, topic, qos) for topic in
                           ('/local_costmap/footprint','/global_costmap/footprint')]
        self.create_subscription(String, '/leader/mapping/control', self.control, 10)
        self.create_subscription(Odometry, '/leader/odometry/local', self.on_odom, 10)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_timer(.5, self.publish_state)
        self.publish_state()
        self.status('Ready: P=HOLD, M=MAPPING, H=HOME, L=LOAD_MAP. No goals sent at startup.')

    def on_odom(self, message):
        self.odom = message
        self.odom_received = self.get_clock().now()

    def status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def payload_dimensions(self):
        front = float(self.get_parameter('payload_front_m').value)
        half = float(self.get_parameter('payload_half_width_m').value)
        return (front,half) if math.isfinite(front) and math.isfinite(half) and front > 0 and half > 0 else None

    def refresh_goal_selection(self):
        enabled = not self.loading and not self.cooperation and (self.mode != 'HOLD' or self.payload_dimensions() is not None)
        if self.selection_value == enabled or self.selection_future is not None or not self.selector_parameters.service_is_ready():
            return
        request = SetParameters.Request()
        request.parameters = [Parameter('enable_nav2_goal_selection', value=enabled).to_parameter_msg()]
        self.selection_future = self.selector_parameters.call_async(request)
        def completed(future):
            self.selection_future = None
            try:
                if all(result.successful for result in future.result().results):
                    self.selection_value = enabled
                else:
                    self.get_logger().error('Could not update Nav2 goal selection')
            except Exception as error:
                self.get_logger().error('Goal selection update failed: '+str(error))
        self.selection_future.add_done_callback(completed)

    def publish_state(self):
        self.refresh_goal_selection()
        self.state_pub.publish(String(data='LOADING' if self.loading else self.mode))
        dimensions = self.payload_dimensions()
        if self.cooperation:
            # Includes both chassis through the PR geometry's reserved +/-12deg hinges.
            # Follower heading can differ by up to 24deg relative to the leader.
            points = [(-.08,-.32),(.75,-.32),(.75,.32),(-.08,.32)]
        elif self.mode == 'HOLD' and dimensions:
            front, half = dimensions
            front,half = max(front,.3175),max(half,.145)
            points = [(-.06,-half),(front,-half),(front,half),(-.06,half)]
        elif self.mode == 'HOLD':
            return  # Never invent an unknown carried-object footprint.
        else:
            points = BASE_FOOTPRINT
        polygon = Polygon(points=[Point32(x=float(x),y=float(y),z=0.) for x,y in points])
        for publisher in self.footprints:
            publisher.publish(polygon)

    def control(self, message):
        command = message.data.upper()
        if self.loading:
            self.status('Map reload in progress; command ignored')
            return
        if command == 'COOP_PREPARE':
            self.prepare_cooperation()
            return
        if command == 'COOP_START':
            if self.cooperation and not self.coop_pending:
                self.coop_pub.publish(String(data='START'))
            else:
                self.status('Press B and wait for COOP READY before N')
            return
        if command == 'COOP_ABORT':
            if self.cooperation:
                self.coop_pub.publish(String(data='ABORT'))
            return
        if command in ('HOLD','MAPPING','HOME','LOAD_MAP') and self.cooperation:
            self.coop_pub.publish(String(data='ABORT'))
            self.cooperation = False
            self.coop_pending = False
        if command in ('HOLD','MAPPING'):
            self.source_pub.publish(String(data='STOP'))
            self.mode = command
            self.publish_state()
            extra = ''
            if command == 'HOLD' and not self.payload_dimensions():
                extra = '; set payload_front_m and payload_half_width_m before navigation'
            self.status(command + extra)
        elif command == 'HOME':
            self.home()
        elif command == 'LOAD_MAP':
            self.load_map()

    def prepare_cooperation(self):
        if self.coop_pending:
            return
        if not self.selector_parameters.service_is_ready():
            self.status('COOP unavailable: selector service missing')
            return
        self.cooperation = True
        self.coop_pending = True
        self.mode = 'HOLD'
        self.source_pub.publish(String(data='STOP'))
        self.publish_state()
        request = SetParameters.Request()
        request.parameters = [Parameter('enable_nav2_goal_selection', value=False).to_parameter_msg()]
        def locked(future):
            self.coop_pending = False
            try:
                if not future.result().results or not all(r.successful for r in future.result().results):
                    raise RuntimeError('Nav2 automatic selection was not disabled')
                self.selection_value = False
                if self.cooperation:
                    self.coop_pub.publish(String(data='PREPARE'))
                    self.status('COOP: B accepted; wait WAIT_PLAN, then select NEW RViz goal. No motion until READY and N.')
            except Exception as error:
                self.status('COOP prepare failed: '+str(error))
        self.selector_parameters.call_async(request).add_done_callback(locked)

    def read_snapshot(self):
        directory = Path(str(self.get_parameter('anchor_directory').value)).resolve()
        manifest = json.loads((directory/'manifest.json').read_text())
        session = str(self.get_parameter('mapping_session_id').value)
        restored = os.environ.get('DAMGC_MAPPING_SNAPSHOT','')
        is_restored = bool(restored) and Path(restored).resolve() == directory
        if not is_restored and (not session or manifest.get('mapping_session_id') != session):
            raise ValueError('Anchor belongs to another map; use run_saved_mapping.sh or save an anchor in this session')
        if manifest['map_frame'] != 'odom':
            raise ValueError('Unsupported anchor frame')
        return directory, manifest

    def load_map(self):
        try:
            directory, manifest = self.read_snapshot()
            path = (directory/manifest['nvblox_file']).resolve()
            if path.parent != directory or not path.is_file() or not path.stat().st_size:
                raise ValueError('Missing/invalid saved map file')
            if not self.load_client.service_is_ready() or not self.selector_parameters.service_is_ready():
                raise ValueError('Map/command selector service unavailable')
        except (OSError,ValueError,KeyError,TypeError) as error:
            self.status('Map reload unavailable: '+str(error))
            return
        self.loading = True
        self.publish_state()
        self.source_pub.publish(String(data='STOP'))
        self.status('Reloading saved map; stopping navigation, preserving current odometry')
        request = SetParameters.Request()
        request.parameters = [Parameter('source_mode', value='STOP').to_parameter_msg()]
        def stopped(future):
            try:
                response = future.result()
                if not response.results or not all(r.successful for r in response.results):
                    raise RuntimeError('Command selector did not accept STOP')
                request = FilePath.Request()
                request.file_path = str(path)
                self.load_client.call_async(request).add_done_callback(loaded)
            except Exception as error:
                self.loading = False
                self.publish_state()
                self.status('Map reload failed: '+str(error))
        def loaded(future):
            try:
                response = future.result()
                self.status('Saved map reloaded; current pose preserved, motors remain STOP' if response.success else 'Map reload failed; motors remain STOP')
            except Exception as error:
                self.status('Map reload failed: '+str(error))
            finally:
                self.loading = False
                self.publish_state()
        self.selector_parameters.call_async(request).add_done_callback(stopped)

    def home(self):
        if self.pending:
            self.status('HOME goal already pending/active')
            return
        if self.mode == 'HOLD' and not self.payload_dimensions():
            self.status('HOME unavailable: carried-object dimensions are unset')
            return
        try:
            directory, manifest = self.read_snapshot()
            anchor = manifest.get('home_anchor', manifest['anchor'])
            x,y,yaw = [float(anchor[k]) for k in ('x','y','yaw_rad')]
            if not all(math.isfinite(v) for v in (x,y,yaw)):
                raise ValueError('Invalid anchor')
            if not self.odom or self.odom.header.frame_id != 'odom' or not self.odom_received or (self.get_clock().now()-self.odom_received).nanoseconds > 1_000_000_000:
                raise ValueError('Local odometry unavailable/stale')
            if not self.nav.server_is_ready():
                raise ValueError('Nav2 action server unavailable')
        except (OSError,ValueError,KeyError,TypeError) as error:
            self.status('HOME unavailable: ' + str(error))
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'odom'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x,goal.pose.pose.position.y = x,y
        goal.pose.pose.orientation.z = math.sin(yaw/2)
        goal.pose.pose.orientation.w = math.cos(yaw/2)
        self.pending = True
        self.status(f'HOME requested: x={x:.3f}, y={y:.3f}, heading={math.degrees(yaw):.1f} deg')
        self.nav.send_goal_async(goal).add_done_callback(self.goal_response)

    def goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.pending = False
                self.status('HOME rejected by Nav2')
                return
            self.status('HOME accepted by Nav2')
            handle.get_result_async().add_done_callback(self.goal_result)
        except Exception as error:
            self.pending = False
            self.status('HOME send failed: '+str(error))

    def goal_result(self, future):
        self.pending = False
        try:
            status = future.result().status
            self.status(f'HOME ended, action status={status}')
        except Exception as error:
            self.status('HOME result failed: '+str(error))


def main():
    rclpy.init()
    node = MappingModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
