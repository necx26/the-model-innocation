#!/usr/bin/env python3
"""Local practice controller. Real camera, Nav2, MoveIt and Gazebo joint attachment."""
import copy
import hashlib
import json
import math
import os
import random
import signal
import subprocess
import threading
import time
import urllib.request
from urllib.parse import urlsplit, parse_qs
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path as FilePath

import cv2
cv2.setNumThreads(1)
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import Twist, PoseStamped, Point, Point32, PolygonStamped
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, JointState, LaserScan, PointCloud2, PointField
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from nav2_msgs.action import NavigateToPose, BackUp, DriveOnHeading
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import ModelStates, LinkStates, EntityState
from gazebo_msgs.srv import SetEntityState
from std_msgs.msg import String, UInt32, Float32, Empty as EmptyMessage
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetStateValidity
from moveit_msgs.msg import Constraints, JointConstraint
from linkattacher_msgs.srv import AttachLink, DetachLink
from tf2_ros import Buffer, TransformListener
from mission_strategy import MissionMap, ObjectMemory, choose_observed_deliveries, delivery_slot
from model_adapter import invoke_official
from scene_config import SCENE, ZONES, PATROLS, in_delivery_area
from exam_bridge import generate_question, validate_exam_config
from cube_perception import floor_cube_rejection, strict_color_candidates, confirmed_near_target, fit_floor_cube_box
from search_memory import SearchMemory, observable_floor_points, scan_clear_points
from robot_geometry import NAV_FOOTPRINT, point_in_footprint, lidar_front_lane_clear, usable_drop_candidates, departure_direction, delivery_region_approaches, square_gripper_yaw, downward_gripper_quaternion, front_corridor_distance, braking_speed
from mission_review import review_markdown
from search_experience import SearchExperience
from robot_geometry import grasp_preflight_valid

BASE = FilePath(os.environ.get('YZBOT_WS') or FilePath(__file__).resolve().parent)
LOG = FilePath(os.environ.get('YZBOT_LOG') or BASE/'logs')
LOG.mkdir(parents=True, exist_ok=True)
JOINTS = [f'joint{i}' for i in range(1, 7)]


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def angle(a):
    return (a + math.pi) % (2*math.pi) - math.pi


class Competition(Node):
    def __init__(self):
        super().__init__('competition_manager', parameter_overrides=[Parameter('use_sim_time', value=True)],
                         automatically_declare_parameters_from_overrides=True)
        self.lock = threading.RLock()
        self.models, self.links, self.joints = {}, {}, {}
        self.image, self.info, self.image_time = None, None, 0.
        self.image_stamp = None
        self.frames = deque(maxlen=30)
        self.camera_jpeg = {}
        self.camera_metrics = {}
        self.recognition_frames = 0
        self.recognition_started = time.monotonic()
        self.pixel_detections = []
        self.last_tf_ok = 0.
        self.last_observed_stamp = None
        self.field = MissionMap(BASE/'src/yzbot/bot_navigation/maps/map.yaml')
        self.memory = ObjectMemory()
        self.search_memory = SearchMemory(self.field.search_points)
        self.search_seen = self.search_memory.seen
        self.search_attempted = {}
        self.observation_enabled = False
        self.perception_paused = False
        self.fixed_camera_images = {'overview': None, 'oblique': None}
        self.clock_time = 0.
        self.scan_min = 99.
        self.latest_scan = None
        self.latest_scan_time = 0.
        self.front_obstacle_range = 99.
        self.front_range_time = 0.
        self.front_range_history = deque(maxlen=3)
        self.last_front_sample = None
        self.front_closing_speed = 0.
        self.forward_speed=0.;self.odom_time=0.
        self.speed_update_time=time.monotonic()
        self.speed_limit_mps = float(SCENE.get('cruise_speed_mps', 1.0))
        self.speed_resume_since = None
        self.speed_policy_label = '巡航'
        self.slam_grid = None
        self.slam_grid_time = 0.
        self.nav_handle = None
        self.navigation_footprints = {}
        self.navigation_metrics = []
        self.navigation_feedback = {}
        self.arm_handle = None
        self.cancel = threading.Event()
        self.busy = False
        self.execution_time_factor = 1.0
        self.started = None
        self.run_id = None
        self.phase_timings = {}
        self.phase_started = time.monotonic()
        self.experience=None
        self.experience_attempted=[]
        self.experience_misses=0
        self.profiled_straight=False
        self.active_target_position=None
        self.observation_mode=False
        self.observation_started=None
        self.used = set()
        self.held_model = None
        self.grasp_attempts=0
        self.grasp_preflight=None
        self.task_mode = os.environ.get('YZBOT_TASK_MODE', 'practice')
        if self.task_mode not in ('practice', 'official', 'exam'):
            raise ValueError('YZBOT_TASK_MODE 必须为 practice、exam 或 official')
        self.official_input = None
        self.exam_config = validate_exam_config(dict(SCENE['exam']))
        self.trace, self.plan = [], []
        self.state = {'mode': '本地练习 · Gazebo 真值定位 · 官方接口待提供', 'phase': '就绪',
                      'status': '等待场景就绪', 'question': '', 'task': None,
                      'picked': 0, 'placed': 0, 'total': 5, 'gripper': '已释放',
                      'events': [], 'detections': [], 'llm': '本地 Qwen 解析约束 + 算术工具精确求解',
                      'result': [], 'running': False, 'elapsed': 0., 'remaining': 300.}
        self.state['task_mode'] = self.task_mode
        self.state['runtime_source_sha256'] = hashlib.sha256(FilePath(__file__).read_bytes()).hexdigest()
        source_files=['competition_node.py','mission_strategy.py','search_memory.py','search_experience.py',
                      'robot_geometry.py','cube_perception.py','mission_review.py','scene_config.json',
                      'src/yzbot/bot_navigation/param/competition_nav2.yaml']
        self.state['runtime_source_files_sha256']={
            'ros_ws/'+name:hashlib.sha256((BASE/name).read_bytes()).hexdigest() for name in source_files}
        self.state['runtime_source_files_sha256'].update({
            'native/'+name:hashlib.sha256((BASE.parent/'native'/name).read_bytes()).hexdigest()
            for name in ('restart-navigation.sh','navigation_process.py','wait_ready.py','start.sh')})
        if self.task_mode == 'official':
            self.state.update(mode='官方模型适配模式 · Gazebo 真值定位 · 接口需配置',
                              status='等待官方题目和已实现的官方模型适配器',
                              llm='官方模型适配器（需对接验证）')
        if self.task_mode == 'exam':
            self.state.update(mode='出题程序考试 · Qwen工具联调 · Gazebo真值定位',
                              status='赛前确认映射规则和顺序，随机复位后点击开始考试', llm='Qwen + 数学工具（本地联调）')
        self.state['exam_config'] = self.exam_config
        self.state['localization']=os.environ.get('YZBOT_LOCALIZATION','truth')
        if self.state['localization']=='slam':self.state['mode']=self.state['mode'].replace('Gazebo真值定位','激光SLAM与仿真里程计')
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(ModelStates, '/demo/model_states_demo', self.model_cb, 10)
        self.create_subscription(LinkStates, '/demo/link_states_demo', self.link_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 30)
        camera_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed',
                                 lambda m: self.compressed_image_cb('front', m), camera_qos)
        self.create_subscription(CameraInfo, '/camera/camera_info', lambda m: setattr(self, 'info', m), camera_qos)
        for view in self.fixed_camera_images:
            self.create_subscription(CompressedImage, f'/fixed_{view}/image_raw/compressed',
                                     lambda m, name=view: self.compressed_image_cb(name, m), camera_qos)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', lambda _: setattr(self, 'clock_time', time.monotonic()), qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', self.plan_cb, 10)
        for costmap in ('local_costmap', 'global_costmap'):
            self.create_subscription(PolygonStamped, '/'+costmap+'/published_footprint',
                                     lambda msg, name=costmap: self.navigation_footprint_cb(name, msg), 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.vel = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.stop_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed_limit_pub = self.create_publisher(SpeedLimit, '/competition/speed_limit', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pubs = {n: self.create_publisher(String, '/competition/'+n, qos)
                     for n in ['task', 'status', 'detections', 'events', 'voice_text']}
        self.telemetry = {}
        for name, kind in [('question',String),('order',String),('red_count',UInt32),('blue_count',UInt32),
                           ('red_zone',String),('blue_zone',String),('progress',UInt32),('work_state',String),
                           ('gripper_state',String),('work_phase',String),('gripper_code',String),('remaining',Float32)]:
            self.telemetry[name] = (self.create_publisher(kind, '/competition/'+name, qos), kind)
        self.annotated_pub = self.create_publisher(Image, '/competition/annotated_image', qos_profile_sensor_data)
        self.rviz_image_pub = self.create_publisher(Image, '/competition/rviz_image', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(EmptyMessage, '/competition/start_exam', lambda _: self.start(), 10)
        self.create_subscription(EmptyMessage, '/competition/stop', lambda _: self.stop(), 10)
        self.create_timer(.2, self.publish_overlay)
        self.command_pub = self.create_publisher(String, '/llama_command', qos)
        self.path_pub = self.create_publisher(Path, '/competition/path', qos)
        initial_path=Path();initial_path.header.frame_id='map';self.path_pub.publish(initial_path)
        self.trace_pub = self.create_publisher(Path, '/competition/traveled_path', qos)
        self.rviz_trace=[]
        self.create_timer(.5,self.publish_rviz_trace)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.arm_action = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.backup_action = ActionClient(self, BackUp, '/backup')
        self.drive_action = ActionClient(self, DriveOnHeading, '/drive_on_heading')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.planner = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.attach = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach = self.create_client(DetachLink, '/DETACHLINK')
        self.reset = self.create_client(Empty, '/reset_world')
        self.set_entity = self.create_client(SetEntityState, '/demo/set_entity_state')
        self.behavior_parameters=self.create_client(GetParameters,'/behavior_server/get_parameters')
        self.clear_costmaps=[self.create_client(ClearEntireCostmap,'/'+name+'/clear_entirely_'+name)
                             for name in ('local_costmap','global_costmap')]
        self.create_timer(.5, self.publish_status)
        self.footprint_pub=self.create_publisher(PolygonStamped,'/competition/footprint',10)
        self.create_timer(.2,self.publish_footprint)
        # Front camera publishes at up to 30 Hz; poll at 100 Hz in sim time.
        # The timestamp guard below still ensures each frame is processed once.
        self.create_timer(.01,self.observe_scene)
        self.create_timer(.1,self.update_dynamic_speed)
        self.placed_cargo_pub=self.create_publisher(PointCloud2,'/competition/placed_cargo',10)
        self.create_timer(.2,self.publish_placed_cargo)

    def publish_placed_cargo(self):
        # Laser rays pass above 5 cm cargo. Protect current camera-confirmed
        # floor objects as well as successful placements; exclude the active
        # pickup, whose final clearance is checked by the visual approach.
        points=[]
        for item in self.memory.snapshot():
            if item['status'] not in ('available','unreachable'):continue
            x,y=item['x'],item['y']
            if self.active_target_position and math.dist((x,y),self.active_target_position)<.4:continue
            points.extend((x+dx,y+dy,.05) for dx in (-.10,-.05,0.,.05,.10)
                          for dy in (-.10,-.05,0.,.05,.10))
        for item in self.state.get('result',[]):
            x,y,_=item['position']
            points.extend((x+dx,y+dy,.05) for dx in (-.035,0.,.035) for dy in (-.035,0.,.035))
        msg=PointCloud2();msg.header.frame_id='odom';msg.header.stamp=self.get_clock().now().to_msg()
        msg.height=1;msg.width=len(points);msg.is_dense=True;msg.point_step=12;msg.row_step=len(points)*12
        msg.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                    for i,name in enumerate(('x','y','z'))]
        msg.data=np.asarray(points,dtype=np.float32).tobytes()
        self.placed_cargo_pub.publish(msg)

    def publish_footprint(self):
        # Body-relative, unstamped footprint remains valid after long arm work.
        msg=PolygonStamped();msg.header.frame_id='base_control'
        msg.polygon.points=[Point32(x=x,y=y) for x,y in NAV_FOOTPRINT]
        self.footprint_pub.publish(msg)

    def navigation_footprint_cb(self, name, msg):
        self.navigation_footprints[name] = {
            'received': time.monotonic(),
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
        }

    def navigation_health(self):
        now = self.get_clock().now().nanoseconds / 1e9
        wall = time.monotonic()
        return {name: {'pose_age': round(max(0., now-value['stamp']), 3),
                       'receive_age': round(wall-value['received'], 3)}
                for name, value in self.navigation_footprints.items()}

    def model_cb(self, msg):
        self.models = dict(zip(msg.name, msg.pose))
        p = self.models.get('six_arm')
        if p:
            xy = [p.position.x, p.position.y]
            if not self.trace or math.dist(xy, self.trace[-1]) > .025:
                self.trace.append(xy)
                self.trace = self.trace[-5000:]

    def link_cb(self, msg):
        self.links = dict(zip(msg.name, msg.pose))

    def joint_cb(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def compressed_image_cb(self, view, msg):
        now = time.monotonic()
        stats = self.camera_metrics.setdefault(view, {'frames': 0, 'advances': 0, 'errors': 0,
                                                       'last_progress': now, 'first': now})
        try:
            data = bytes(msg.data)
            stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
            if stamp != stats.get('stamp'):
                stats['advances'] += 1
                stats['last_progress'] = now
            stats.update(frames=stats['frames']+1, stamp=stamp, last_receive=now, bytes=len(data))
            self.camera_jpeg[view] = data
            if view != 'front':return
            image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None: raise ValueError('JPEG 解码失败')
            self.image = image
            self.image_time = stats['last_progress']
            self.image_stamp = msg.header.stamp
            self.frames.append((image, msg.header.stamp, now))
        except Exception as error:
            stats['errors'] += 1
            stats['error'] = str(error)

    def camera_diagnostics(self):
        now = time.monotonic()
        result = {}
        for name in ('front', 'overview', 'oblique'):
            stats = self.camera_metrics.get(name, {})
            result[name] = dict(frames=stats.get('frames', 0), advances=stats.get('advances', 0),
                                age=round(now-stats.get('last_progress', now-999), 2),
                                receive_age=round(now-stats.get('last_receive', now-999), 2),
                                fps=round(stats.get('advances', 0)/max(.1, now-stats.get('first', now)), 2),
                                bytes=stats.get('bytes', 0), errors=stats.get('errors', 0),
                                stamp_ns=stats.get('stamp'), transport='reliable JPEG')
        result['front']['recognition_frames']=self.recognition_frames
        result['front']['recognition_fps']=round(self.recognition_frames/max(.1,now-self.recognition_started),2)
        return result

    def color_candidates(self, image):
        return strict_color_candidates(image)

    def publish_overlay(self):
        publishers=[(self.annotated_pub,None),(self.rviz_image_pub,(320,180))]
        active=[(publisher,size) for publisher,size in publishers if publisher.get_subscription_count()]
        if self.image is None or self.image_stamp is None or not active:return
        img=self.image.copy()
        for d in self.pixel_detections:
            if d.get('reason') != '可抓取目标':continue
            x,y,w,h=d['box'];cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.putText(img,d['color'],(x,max(14,y-4)),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,0),1)
        for publisher,size in active:
            pixels=cv2.resize(img,size,interpolation=cv2.INTER_AREA) if size else img
            msg=Image();msg.header.stamp=self.image_stamp;msg.header.frame_id='camera_link'
            msg.height,msg.width=pixels.shape[:2];msg.encoding='bgr8';msg.step=msg.width*3;msg.data=pixels.tobytes()
            publisher.publish(msg)

    def publish_rviz_trace(self):
        pose=self.robot_pose()
        if pose and (not self.rviz_trace or math.dist(pose[:2],self.rviz_trace[-1])>.04):
            self.rviz_trace.append(pose[:2]);self.rviz_trace=self.rviz_trace[-5000:]
        msg=Path();msg.header.frame_id='odom';msg.header.stamp=Time().to_msg()
        step=max(1,math.ceil(len(self.rviz_trace)/600))
        for x,y in self.rviz_trace[::step]:
            p=PoseStamped();p.header=msg.header;p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.w=1.
            msg.poses.append(p)
        self.trace_pub.publish(msg)

    def observe_scene(self):
        if self.image_stamp is None:return
        try:
            current=(self.image_stamp.sec,self.image_stamp.nanosec)
            if current==self.last_observed_stamp:return
            detections,tf,image_stamp=self.vision(with_pose=True)
            stamp=(image_stamp.sec,image_stamp.nanosec)
            if stamp==self.last_observed_stamp:return
            self.last_observed_stamp=stamp
            self.recognition_frames+=1
            if not self.observation_enabled or self.perception_paused:return
            # The arm is stowed and body pixels are rejected by vision(). Do not
            # discard every other object of the carried color along the route.
            if self.held_model:
                rp=self.robot_pose()
                if rp:detections=[d for d in detections if math.hypot(d['x']-rp[0],d['y']-rp[1])>.65]
            for target in self.memory.update(detections):
                self.event('相机发现并记住目标',target_id=target['id'],color=target['color'],
                           position=[target['x'],target['y']],source='onboard_camera')
            image=next((f[0] for f in reversed(self.frames)
                        if (f[1].sec,f[1].nanosec)==stamp),None)
            if image is None or self.info is None:return
            org=np.array([tf.translation.x,tf.translation.y,tf.translation.z])
            indices=observable_floor_points(self.field.search_points,org,rotation(tf.rotation),
                self.info.k,image,self.field.line_free,detections)
            scan=self.latest_scan
            if scan is None or time.monotonic()-self.latest_scan_time>.75:return
            laser=self.buffer.lookup_transform('odom',scan.header.frame_id,Time.from_msg(scan.header.stamp)).transform
            q=laser.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            clear=scan_clear_points(self.field.search_points[indices],
                [laser.translation.x,laser.translation.y],yaw,scan.ranges,
                scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max)
            self.search_memory.observe(indices[clear],detections,stamp)
        except Exception as e:
            self.state['perception_note']=str(e)

    def scan_cb(self, msg):
        self.latest_scan=msg;self.latest_scan_time=time.monotonic()
        valid=[v for v in msg.ranges if math.isfinite(v) and msg.range_min < v < msg.range_max]
        self.scan_min=min(valid,default=99.)
        distance=front_corridor_distance(msg)
        if distance is None:return
        now=time.monotonic()
        self.front_range_history.append(distance)
        # Never smooth away a newly appearing obstacle. Median only damps the
        # closing-speed estimate; braking uses the latest conservative range.
        filtered=float(np.median(self.front_range_history))
        if self.last_front_sample:
            previous_time,previous_distance=self.last_front_sample
            dt=now-previous_time
            if .03<dt<1.0:
                raw_closing=(previous_distance-filtered)/dt
                # Patrol vehicles are slow relative to the 1 m/s base; cap scan-rate
                # spikes caused by turning past walls so they cannot create false TTCs.
                closing_cap=float(SCENE.get('cruise_speed_mps',1.5))+.5
                raw_closing=max(-closing_cap,min(closing_cap,raw_closing))
                self.front_closing_speed=.65*self.front_closing_speed+.35*raw_closing
        self.last_front_sample=(now,filtered)
        self.front_obstacle_range=min(distance,filtered)
        self.front_range_time=now

    def odom_cb(self,msg):
        self.forward_speed=float(msg.twist.twist.linear.x)
        self.odom_time=time.monotonic()

    def lidar_lane_clear(self, distance):
        """Check the swept front corridor with fresh lidar after facing target."""
        scan=self.latest_scan
        if scan is None or time.monotonic()-self.latest_scan_time>.5:return False
        return lidar_front_lane_clear(scan,distance)

    def update_dynamic_speed(self):
        """Continuous stopping envelope, in addition to Nav2 collision checking."""
        now=time.monotonic();dt=min(.3,max(0.,now-self.speed_update_time))
        self.speed_update_time=now
        cruise=float(SCENE.get('cruise_speed_mps',1.0))
        active=bool(self.busy and (self.nav_handle is not None or self.observation_mode))
        age=max(0.,now-self.front_range_time)
        own_speed=max(0.,self.forward_speed) if now-self.odom_time<.5 else 0.
        distance=self.front_obstacle_range
        closing=max(0.,self.front_closing_speed)
        # Five-centimetre cargo is below the laser. Apply the same swept lane
        # test to genuine visual tracks, retaining the laser-origin convention.
        pose=self.robot_pose()
        if active and pose:
            for target in self.memory.snapshot():
                if target['status'] not in ('available','unreachable'):continue
                if self.active_target_position and math.dist((target['x'],target['y']),self.active_target_position)<.4:continue
                dx,dy=target['x']-pose[0],target['y']-pose[1]
                forward=math.cos(pose[2])*dx+math.sin(pose[2])*dy
                lateral=-math.sin(pose[2])*dx+math.cos(pose[2])*dy
                if forward>0 and abs(lateral)<.30 and forward+.10<distance:
                    distance=max(.01,forward+.10);closing=max(closing,own_speed)
        external_closing=max(0.,closing-own_speed)
        clearance=max(0.,distance-.30-closing*min(age,.75))
        proposed=cruise
        label='巡航/安全通行'
        if active and age>.75:
            proposed=min(.25,cruise);label='雷达数据过期，保守限速'
        elif active:
            proposed=min(cruise,braking_speed(clearance,external_closing,reaction=.25+age))
        # Nav2 reserves a zero SpeedLimit for UNLIMITED. A positive epsilon
        # requests a practical stop, with the collision checker still active.
        proposed=max(.001,proposed)
        self.speed_limit_mps=min(proposed,self.speed_limit_mps+1.5*dt)
        if active and age<=.75:
            if self.speed_limit_mps<=.12:label='紧急减速/等待'
            elif self.speed_limit_mps<=.35:label='减速让行'
            elif self.speed_limit_mps<cruise-.1:label='按停车距离通过'
        old_label=self.state.get('dynamic_obstacle_policy')
        old_limit=self.state.get('dynamic_speed_limit_mps')
        self.state['dynamic_obstacle_policy']=label
        self.state['dynamic_speed_limit_mps']=round(self.speed_limit_mps,3)
        self.state['dynamic_front_range_m']=round(distance,2) if distance<90 else None
        self.state['dynamic_closing_speed_mps']=round(self.front_closing_speed,2)
        self.state['dynamic_bumper_clearance_m']=round(clearance,3)
        self.state['dynamic_external_closing_mps']=round(external_closing,3)
        ttc=distance/closing if closing>.05 and distance<90 else float('inf')
        self.state['dynamic_ttc_s']=round(ttc,2) if math.isfinite(ttc) else None
        if label!=old_label or old_limit is None or abs(self.speed_limit_mps-float(old_limit))>=.15:
            self.event('动态障碍速度决策',decision=label,speed_limit_mps=round(self.speed_limit_mps,3),
                       front_range_m=round(distance,2),bumper_clearance_m=round(clearance,3),
                       external_closing_mps=round(external_closing,3))
        message=SpeedLimit()
        message.header.stamp=self.get_clock().now().to_msg()
        message.percentage=False
        message.speed_limit=float(self.speed_limit_mps)
        self.speed_limit_pub.publish(message)

    def map_cb(self, msg):
        self.slam_grid = msg
        self.slam_grid_time = time.monotonic()

    def map_view(self):
        """Real ROS /map, with odom overlays transformed into the map frame."""
        import base64
        msg = self.slam_grid
        if msg is None:
            raise ValueError('等待 ROS /map 数据')
        info = msg.info
        values = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        pixels = np.where(values < 0, 185, 255 - np.clip(values, 0, 100) * 2.55).astype(np.uint8)
        png = cv2.imencode('.png', np.flipud(pixels))[1].tobytes()
        tf = self.buffer.lookup_transform(msg.header.frame_id, 'odom', Time()).transform
        q = tf.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return dict(image='data:image/png;base64,'+base64.b64encode(png).decode(),
                    frame=msg.header.frame_id, width=info.width, height=info.height,
                    resolution=info.resolution,
                    origin=[info.origin.position.x, info.origin.position.y],
                    origin_yaw=math.atan2(2*(info.origin.orientation.w*info.origin.orientation.z+
                                           info.origin.orientation.x*info.origin.orientation.y),
                                          1-2*(info.origin.orientation.y**2+info.origin.orientation.z**2)),
                    map_from_odom=[tf.translation.x, tf.translation.y, yaw],
                    age=round(time.monotonic()-self.slam_grid_time, 1),
                    known_cells=int(np.count_nonzero(values >= 0)))

    def plan_cb(self, msg):
        self.plan = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        try:
            t=self.buffer.lookup_transform('odom',msg.header.frame_id,Time()).transform
            rot=rotation(t.rotation)
            self.plan=[(rot@np.array([x,y,0.])+np.array([t.translation.x,t.translation.y,t.translation.z]))[:2].tolist() for x,y in self.plan]
        except Exception:pass
        self.path_pub.publish(msg)

    def emit(self, topic, value):
        self.pubs[topic].publish(String(data=json.dumps(value, ensure_ascii=False)))

    def event(self, text, **extra):
        entry = {'time': round(self.elapsed(), 2), 'text': text, **extra}
        with self.lock:
            self.state['events'].append(entry)
            self.state['events'] = self.state['events'][-150:]
        self.emit('events', entry)
        self.emit('voice_text', {'text': text, 'time': entry['time']})
        with (LOG/'competition-events.jsonl').open('a') as f:
            f.write(json.dumps(entry, ensure_ascii=False)+'\n')
        if self.run_id:
            with (LOG/'runs'/self.run_id/'events.jsonl').open('a') as f:
                f.write(json.dumps(entry,ensure_ascii=False)+'\n')
        self.get_logger().info(text)

    def update(self, phase, text):
        if self.started and phase!=self.state['phase']:
            previous=self.state['phase']
            self.phase_timings[previous]=self.phase_timings.get(previous,0)+time.monotonic()-self.phase_started
            self.phase_started=time.monotonic()
        self.state.update(phase=phase, status=text)
        self.event(text)

    def elapsed(self):
        return time.monotonic()-self.started if self.started else 0.

    def snapshot(self):
        with self.lock:
            s = copy.deepcopy(self.state)
        if self.busy:
            s['elapsed'] = round(self.elapsed(), 1)
            s['remaining'] = max(0., round(300-self.elapsed(), 1))
        s.update(busy=self.busy, path=self.trace[-2500:], plan=self.plan, scan_min=round(self.scan_min, 3),
                 known_targets=self.memory.snapshot(),
                 search_coverage=round(100*float(np.mean(self.search_seen)),1),
                 search_memory=self.search_memory.summary(), execution_time_factor=round(self.execution_time_factor, 2),
                 search_bounds=self.field.bounds, zones=ZONES, patrols=PATROLS, exam_config=self.exam_config,
                 camera_age=round(time.monotonic()-self.image_time, 1),
                 cameras=self.camera_diagnostics(), pixel_detections=copy.deepcopy(self.pixel_detections),
                 tf_camera_age=round(time.monotonic()-self.last_tf_ok, 2),
                 robot=self.robot_pose(),
                 navigation_health=self.navigation_health(),
                 navigation_metrics=copy.deepcopy(self.navigation_metrics),
                 navigation_feedback=copy.deepcopy(self.navigation_feedback),
                 phase_seconds={k:round(v,2) for k,v in self.phase_timings.items()},
                 grasp_attempts=self.grasp_attempts,
                 run_id=self.run_id,
                 observation_mode=self.observation_mode,stop_requested=self.cancel.is_set(),
                 cubes=[{'name': n, 'x': p.position.x, 'y': p.position.y, 'z': p.position.z}
                        for n, p in self.models.items() if n.startswith(('red_cube', 'blue_cube'))],
                 obstacles=[{'name': n, 'x': p.position.x, 'y': p.position.y}
                            for n, p in self.models.items() if n in ['Move1', 'Move2']])
        issues=[]
        if s['robot'] is None:issues.append('缺少 odom→base_control 坐标变换')
        if self.image is None or s['camera_age']>3:issues.append('前置相机没有新图像')
        if time.monotonic()-self.last_tf_ok>3:issues.append('相机帧与 TF 尚未同步')
        for view in ('overview', 'oblique'):
            if s['cameras'][view]['age']>3:issues.append(view+' 固定相机没有新图像')
        if time.monotonic()-self.clock_time>3:issues.append('Gazebo 仿真时钟停止')
        if self.state['localization']=='slam' and (self.slam_grid is None or time.monotonic()-self.slam_grid_time>5):issues.append('SLAM 地图没有更新')
        for name, health in s['navigation_health'].items():
            if health['pose_age']>2 or health['receive_age']>2:
                issues.append(name+' 导航底盘位置未实时更新')
        s.update(healthy=not issues,health_issues=issues,runtime_project=str(BASE.parent))
        return s

    def publish_status(self):
        task = self.state.get('task') or {}
        data = dict(question=self.state.get('question',''), order=('红→蓝' if task.get('first')=='red' else '蓝→红') if task else '',
                    red_count=task.get('red',0), blue_count=task.get('blue',0),
                    red_zone=task.get('red_zone','待解题'), blue_zone=task.get('blue_zone','待解题'),
                    progress=self.state['placed'], work_state=self.state['phase']+'：'+self.state['status'],
                    gripper_state=self.state['gripper'], work_phase=self.state['phase'],
                    gripper_code='holding' if self.held_model else ('closed' if self.state['gripper']=='闭合' else 'released'), remaining=float(max(0,300-self.elapsed())) if self.started else 300.)
        for name,(publisher,kind) in self.telemetry.items():publisher.publish(kind(data=data[name]))
        self.emit('status', {k:v for k,v in self.snapshot().items() if k not in ['events','path','plan','cubes']})

    def halt_motion(self):
        self.vel.publish(Twist());self.stop_vel.publish(Twist())
        for handle in (self.nav_handle,self.arm_handle):
            if handle:
                try:handle.cancel_goal_async()
                except Exception:pass

    def check(self):
        reason=None
        if self.cancel.is_set():reason='任务已停止'
        elif self.started and self.elapsed()>=300:reason='已达到 5 分钟时限，停止作业'
        elif self.started and self.observation_enabled:
            if time.monotonic()-self.image_time>5:reason='前置相机超过 5 秒没有新帧，已停止运动；查看相机诊断'
            elif time.monotonic()-self.clock_time>5:reason='仿真时钟超过 5 秒未更新，已停止运动'
            elif time.monotonic()-self.last_tf_ok>5:reason='图像与 TF 超过 5 秒未同步，已停止运动'
            elif self.nav_handle is not None:
                stale=[name for name, health in self.navigation_health().items()
                       if health['pose_age']>2. or health['receive_age']>2.]
                if stale:reason='导航底盘位置冻结，已停止运动: '+','.join(stale)
        if reason:
            self.halt_motion()
            raise RuntimeError(reason)

    def wait(self, future, timeout=12):
        deadline = time.monotonic()+timeout
        while not future.done():
            self.check()
            if time.monotonic() > deadline:
                raise TimeoutError('ROS 接口等待超时')
            time.sleep(.03)
        result = future.result()
        if result is None:
            raise RuntimeError(str(future.exception()))
        return result

    def call(self, client, request, timeout=12):
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('服务不可用: '+client.srv_name)
        return self.wait(client.call_async(request), timeout)

    def robot_pose(self):
        try:
            t = self.buffer.lookup_transform('odom', 'base_control', Time()).transform
            q = t.rotation
            return [t.translation.x, t.translation.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
        except Exception:
            return None

    def recover_turning_clearance(self):
        """One bounded, collision-checked translation before trying to turn again."""
        self.check()
        options=[]
        if time.monotonic()-self.front_range_time<.75 and self.front_obstacle_range>.85:
            options.append((self.drive_action,DriveOnHeading,.30,'前进'))
        options.append((self.backup_action,BackUp,.20,'后退'))
        for client,kind,distance,label in options:
            self.check()
            if not client.wait_for_server(timeout_sec=1):continue
            goal=kind.Goal(target=Point(x=distance),speed=.15,time_allowance=Duration(sec=6))
            handle=self.wait(client.send_goal_async(goal),4)
            if not handle.accepted:continue
            self.nav_handle=handle
            pending=handle.get_result_async()
            try:result=self.wait(pending,8)
            finally:
                if not pending.done():handle.cancel_goal_async()
                self.nav_handle=None
            self.event('转身空间恢复',direction=label,distance=distance,status=result.status)
            if result.status==4:return True
        return False

    def go(self, x, y, yaw, look_for=None, align_tolerance=.045, align_yaw_tolerance=.035, arrive_region=None):
        try:return self._go_once(x,y,yaw,look_for,align_tolerance,align_yaw_tolerance,arrive_region)
        except (RuntimeError,TimeoutError) as error:
            self.check()
            self.event('导航受阻，检查转身空间',reason=str(error))
            if not self.recover_turning_clearance():raise
            return self._go_once(x,y,yaw,look_for,align_tolerance,align_yaw_tolerance,arrive_region)

    def _go_once(self, x, y, yaw, look_for=None, align_tolerance=.045, align_yaw_tolerance=.035, arrive_region=None):
        self.check()
        if not self.nav.wait_for_server(timeout_sec=5):
            raise RuntimeError('Nav2 action server 不可用')
        goal = NavigateToPose.Goal()
        # World-source odometry shares the competition world's coordinates.
        # Transform the target now, then send stamp=0 to avoid stale TF times.
        transform=self.buffer.lookup_transform('map','odom',Time()).transform
        point=rotation(transform.rotation)@np.array([x,y,0.])+np.array([transform.translation.x,transform.translation.y,transform.translation.z])
        q=transform.rotation
        heading=yaw+math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(point[0]), float(point[1])
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        def feedback(message):
            value=message.feedback
            self.navigation_feedback={'remaining_distance': round(value.distance_remaining,3),
                                      'recoveries': value.number_of_recoveries}
        self.nav_handle = self.wait(self.nav.send_goal_async(goal, feedback_callback=feedback), 10)
        if not self.nav_handle.accepted:
            raise RuntimeError('Nav2 拒绝目标')
        handle=self.nav_handle
        future=handle.get_result_async()
        end=time.monotonic()+65
        progress_pose=self.robot_pose();progress_time=time.monotonic()
        navigation_started=time.monotonic();previous_pose=progress_pose
        travelled=0.;turned=0.;stationary_turn=0.;outcome='cancelled'
        try:
            while not future.done():
                self.check()
                if look_for and any(self.field.approaches(t) for t in self.memory.available(look_for)):
                    self.wait(handle.cancel_goal_async(),5)
                    self.wait(future,8)
                    self.vel.publish(Twist())
                    return False
                pose=self.robot_pose()
                if arrive_region and pose:
                    zone,preferred,occupied=arrive_region
                    if math.dist(pose[:2],zone)<.95 and usable_drop_candidates(self.field,zone,preferred,pose,occupied):
                        self.wait(handle.cancel_goal_async(),3);self.wait(future,4)
                        self.vel.publish(Twist());outcome='region_reachable'
                        self.event('当前姿态已有安全区内落点，结束导航',zone=list(zone))
                        return True
                if pose and previous_pose:
                    travelled+=math.dist(pose[:2],previous_pose[:2])
                    delta=abs(angle(pose[2]-previous_pose[2]))
                    turned+=delta;stationary_turn+=delta
                previous_pose=pose
                if pose and progress_pose and math.dist(pose[:2],progress_pose[:2])>.06:
                    progress_pose=pose;progress_time=time.monotonic();stationary_turn=0.
                elif time.monotonic()-progress_time>7. or stationary_turn>1.5*math.pi:
                    self.wait(handle.cancel_goal_async(),3)
                    self.wait(future,4)
                    raise RuntimeError('导航没有平移进展或反复转向，已取消当前路径')
                if time.monotonic()>end:raise TimeoutError('导航到达超时')
                time.sleep(.1)
            result=future.result()
            outcome='arrived' if result.status==4 else 'action_status_'+str(result.status)
        except Exception as error:
            outcome=str(error)
            raise
        finally:
            if not future.done():handle.cancel_goal_async()
            self.nav_handle=None
            metric=dict(goal=[round(x,3),round(y,3),round(yaw,3)],
                        seconds=round(time.monotonic()-navigation_started,2),
                        travelled_m=round(travelled,3),turned_rad=round(turned,3),
                        outcome=outcome,held=bool(self.held_model))
            self.navigation_metrics.append(metric)
            self.navigation_metrics=self.navigation_metrics[-100:]
        if result.status != 4:
            raise RuntimeError(f'Nav2 未到达目标，状态 {result.status}')
        self.align(x, y, yaw, position_tolerance=align_tolerance, yaw_tolerance=align_yaw_tolerance)
        return True

    def align(self, x, y, yaw, position_tolerance=.045, yaw_tolerance=.035):
        end = time.monotonic()+25
        while time.monotonic() < end:
            self.check()
            p = self.robot_pose()
            if p is None:
                time.sleep(.1)
                continue
            d = math.hypot(x-p[0], y-p[1])
            target = math.atan2(y-p[1], x-p[0]) if d > position_tolerance else yaw
            e = angle(target-p[2])
            direction=1.
            if d>position_tolerance and abs(e)>math.pi/2:
                direction=-1.;e=angle(e+math.pi)
            if d < position_tolerance and abs(angle(yaw-p[2])) < yaw_tolerance:
                self.vel.publish(Twist())
                return
            cmd = Twist()
            cmd.angular.z = float(max(-.8, min(.8, 2.5*e)))
            if d > position_tolerance and abs(e) < .25:
                cmd.linear.x = float(direction*min(.24, max(.04, d*.9)))
            self.vel.publish(cmd)
            time.sleep(.05)
        self.vel.publish(Twist())
        raise RuntimeError('近距离定位超时')

    def robot_state(self):
        from moveit_msgs.msg import RobotState
        s = RobotState()
        s.joint_state.name = list(self.joints)
        s.joint_state.position = list(self.joints.values())
        s.is_diff = False
        return s

    def solve_ik(self, x, y, z, max_seeds=8, world_yaw=None):
        t = self.buffer.lookup_transform('base_footprint', 'odom', Time()).transform
        p = rotation(t.rotation)@np.array([x,y,z])+np.array([t.translation.x,t.translation.y,t.translation.z])
        radial=math.hypot(p[0],p[1]); height=p[2]+.06-.2834
        cosine=(radial*radial+height*height-.14**2-.28**2)/(2*.14*.28)
        analytic=None
        base_yaw=self.robot_pose()[2] if world_yaw is not None else 0.
        wrist_offset=angle(base_yaw-world_yaw) if world_yaw is not None else 0.
        if -1<cosine<1:
            elbow=math.acos(cosine)
            shoulder=math.atan2(radial,height)-math.atan2(.28*math.sin(elbow),.14+.28*math.cos(elbow))
            phi=math.atan2(p[1],p[0])
            analytic=[phi,shoulder,elbow,0.,shoulder+elbow-math.pi,angle(phi+wrist_offset)]
        for seed in range(max_seeds):
            req = GetPositionIK.Request()
            req.ik_request.group_name = 'arm'
            req.ik_request.ik_link_name = 'grasping_frame'
            req.ik_request.robot_state = self.robot_state()
            if analytic is not None and seed==0:
                for j in range(6):
                    idx=req.ik_request.robot_state.joint_state.name.index(JOINTS[j])
                    req.ik_request.robot_state.joint_state.position[idx]=float(analytic[j])
            elif seed>1:
                vals = analytic or [0., 1.3, 1.0, 0., -.8, 0.]
                for j in range(6):
                    idx=req.ik_request.robot_state.joint_state.name.index(JOINTS[j])
                    req.ik_request.robot_state.joint_state.position[idx]=vals[j]+random.uniform(-.5,.5)
            req.ik_request.avoid_collisions = True
            req.ik_request.pose_stamped.header.frame_id = 'base_footprint'
            pose=req.ik_request.pose_stamped.pose
            pose.position.x,pose.position.y,pose.position.z=map(float,p)
            if world_yaw is None:pose.orientation.y=1.;pose.orientation.w=0.
            else:
                (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)=downward_gripper_quaternion(base_yaw,world_yaw)
            req.ik_request.timeout=Duration(sec=1)
            r=self.call(self.ik,req,5)
            if r.error_code.val==1:
                data=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position))
                return [data[j] for j in JOINTS]
        raise RuntimeError(f'MoveIt IK 无可行解: {p.round(3).tolist()}')

    def arm(self, values, speed=1.0):
        req=GetMotionPlan.Request()
        r=req.motion_plan_request
        r.group_name='arm'; r.start_state=self.robot_state()
        r.allowed_planning_time=3.; r.num_planning_attempts=5
        r.max_velocity_scaling_factor=speed; r.max_acceleration_scaling_factor=speed
        c=Constraints()
        for j,v in zip(JOINTS,values):
            c.joint_constraints.append(JointConstraint(joint_name=j,position=float(v),tolerance_above=.002,tolerance_below=.002,weight=1.))
        r.goal_constraints=[c]
        result=self.call(self.planner,req,10).motion_plan_response
        if result.error_code.val!=1:
            raise RuntimeError(f'MoveIt 规划失败: {result.error_code.val}')
        traj=result.trajectory.joint_trajectory
        traj.header.stamp=Time().to_msg()
        if not self.arm_action.wait_for_server(timeout_sec=2):raise RuntimeError('机械臂动作控制器不可用')
        goal=FollowJointTrajectory.Goal(trajectory=traj)
        handle=self.wait(self.arm_action.send_goal_async(goal),5)
        if not handle.accepted:raise RuntimeError('机械臂控制器拒绝轨迹')
        self.arm_handle=handle
        pending=handle.get_result_async()
        duration=traj.points[-1].time_from_start
        deadline=time.monotonic()+max(8,duration.sec+duration.nanosec/1e9+5)
        stable=0
        while time.monotonic()<deadline:
            self.check()
            err=max(abs(self.joints.get(j,99)-v) for j,v in zip(JOINTS,values))
            if pending.done():
                outcome=pending.result()
                if outcome.status==4 and err<.012:
                    self.arm_handle=None;return
                if outcome.status!=4:raise RuntimeError('机械臂动作失败：'+outcome.result.error_string)
            stable=stable+1 if err<.008 else 0
            if stable>=5:
                self.arm_handle=None;return
            time.sleep(.08)
        raise RuntimeError(f'机械臂跟踪未达到目标，最大关节误差 {err:.3f}')

    def arm_xyz(self,x,y,z,speed=.9,world_yaw=None):
        self.arm(self.solve_ik(x,y,z,world_yaw=world_yaw),speed=speed)

    def release_ik(self, x, y, max_seeds=8):
        """Reject unreachable/under-chassis drops before costly IK retries."""
        pose=self.robot_pose()
        if pose is None:raise RuntimeError('放置前没有底盘定位')
        dx,dy=x-pose[0],y-pose[1]
        c,s=math.cos(pose[2]),math.sin(pose[2])
        local=(c*dx+s*dy,-s*dx+c*dy)
        clearance=-cv2.pointPolygonTest(np.asarray(NAV_FOOTPRINT,dtype=np.float32),local,True)
        radial=math.hypot(local[0]+.14,local[1])
        if clearance<.06 or not .16<radial<.40:
            raise RuntimeError('当前放置姿态需要调整：末端工作空间或底盘间距不足')
        return self.solve_ik(x,y,.23,max_seeds),self.solve_ik(x,y,.12,max_seeds)

    def settle_base(self):
        last=None;stable=0
        for _ in range(25):
            self.check();self.vel.publish(Twist());self.stop_vel.publish(Twist())
            pose=self.robot_pose()
            if last and pose and math.dist(pose[:2],last[:2])<.002 and abs(angle(pose[2]-last[2]))<.01:
                stable+=1
                if stable>=3:return
            else:stable=0
            last=pose;time.sleep(.1)
        raise RuntimeError('底盘未停稳，禁止机械臂接近货物')

    def retreat_after_drop(self, landing):
        """After stowing, leave in the direction away from the actual landing."""
        reverse=departure_direction(self.robot_pose(),landing)<0
        client,kind=(self.backup_action,BackUp) if reverse else (self.drive_action,DriveOnHeading)
        if not client.wait_for_server(timeout_sec=2):
            self.event('放置后未能退出',reason='Nav2 直行行为不可用')
            return False
        for distance in (.30, .15):
            if self.cancel.is_set() or (self.started and self.elapsed() >= 300):
                return False
            try:
                goal=kind.Goal(target=Point(x=distance),speed=.30,
                                 time_allowance=Duration(sec=8))
                handle=self.wait(client.send_goal_async(goal),5)
                self.nav_handle=handle
                if not handle.accepted:
                    self.event('放置后后退被拒绝',distance=distance)
                    self.nav_handle=None
                    continue
                result=self.wait(handle.get_result_async(),10)
                self.nav_handle=None
                if result.status==4:
                    self.event('放置后安全后退' if reverse else '放置后向前离开后方落点',
                               distance=distance,speed=.30)
                    return True
                self.event('放置后后退未完成',distance=distance,status=result.status)
            except Exception as e:
                if self.nav_handle:
                    try:self.nav_handle.cancel_goal_async()
                    except Exception:pass
                self.nav_handle=None
                if self.cancel.is_set() or (self.started and self.elapsed() >= 300):
                    return False
                self.event('放置后后退未完成',distance=distance,reason=str(e))
        return False

    def refine_docking_distance(self,target):
        """Correct centimetre-scale axial stopping error through Nav2 behavior.

        Larger/lateral/yaw errors belong to navigation, not a direct servo.
        Both forward and reverse behaviors retain their costmap checks.
        """
        pose=self.robot_pose()
        if pose is None:return False
        dx,dy=target[0]-pose[0],target[1]-pose[1]
        forward=math.cos(pose[2])*dx+math.sin(pose[2])*dy
        lateral=-math.sin(pose[2])*dx+math.cos(pose[2])*dy
        if abs(angle(target[2]-pose[2]))>.15 or abs(lateral)>.04 or not .008<abs(forward)<.08:
            return False
        client,kind=(self.drive_action,DriveOnHeading) if forward>0 else (self.backup_action,BackUp)
        if not client.wait_for_server(timeout_sec=2):return False
        pending=None;handle=None
        try:
            goal=kind.Goal(target=Point(x=abs(forward)),speed=.10,time_allowance=Duration(sec=4))
            handle=self.wait(client.send_goal_async(goal),5)
            if not handle.accepted:return False
            self.nav_handle=handle;pending=handle.get_result_async()
            result=self.wait(pending,6)
            if result.status!=4:return False
            self.settle_base()
            self.event('校正厘米级停靠距离',distance=forward,lateral_error=lateral,pose=list(self.robot_pose()))
            return True
        finally:
            if handle is not None and pending is not None and not pending.done():
                handle.cancel_goal_async()
            self.nav_handle=None

    def grip(self, closed):
        m=JointTrajectory(joint_names=['finger_joint1'])
        m.points=[JointTrajectoryPoint(positions=[.015 if closed else 0.],time_from_start=Duration(nanosec=500_000_000))]
        self.grip_pub.publish(m)
        time.sleep(.55)
        self.state['gripper']='闭合' if closed else '张开'

    def vision(self, with_pose=False):
        if self.image is None or time.monotonic()-self.image_time>3:
            self.state['detections']=[]
            raise RuntimeError('相机没有持续更新的图像')
        # Pixel recognition works independently of TF, including before an exam.
        pixel_candidates=self.color_candidates(self.image)
        self.pixel_detections=pixel_candidates
        if self.info is None:raise RuntimeError('已收到图像，缺少 camera_info 内参')
        frame=None
        for candidate in reversed(list(self.frames)):
            if time.monotonic()-candidate[2]<2 and self.buffer.can_transform('odom','camera_link',Time.from_msg(candidate[1])):
                frame=candidate;break
        if frame is None:
            self.state['detections']=[]
            raise RuntimeError('图像正常，等待与图像时间戳一致的 odom→camera_link TF')
        img,image_stamp,_=frame
        candidates=pixel_candidates if img is self.image else self.color_candidates(img)
        tf=self.buffer.lookup_transform('odom','camera_link',Time.from_msg(image_stamp)).transform
        self.last_tf_ok=time.monotonic()
        rot=rotation(tf.rotation);org=np.array([tf.translation.x,tf.translation.y,tf.translation.z])
        base=self.buffer.lookup_transform('odom','base_control',Time.from_msg(image_stamp)).transform
        q=base.rotation
        rp=[base.translation.x,base.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
        k=self.info.k;result=[]
        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        for d in candidates:
            if self.perception_paused:
                d['reason']='机械臂作业中，暂停目标入库';continue
            x,y,w,h=d['box'];u,v=x+w/2,y+h-1
            if y+h>=img.shape[0]-1:d['reason']='目标底部超出画面';continue
            ray=rot@np.array([1.,-(u-k[2])/k[0],-(v-k[5])/k[4]])
            if ray[2]>=-.015:d['reason']='地面投影无交点';continue
            point=org+ray*((0.-org[2])/ray[2])
            axis=int(np.argmax(np.abs(point[:2]-org[:2])))
            point[axis]+=.025*np.sign(point[axis]-org[axis])
            if np.linalg.norm(point[:2]-org[:2])>5.0:d['reason']='超过 5 米搜索识别范围';continue
            forward=float((rot.T@(point-org))[0])
            d['ground_projection']=[float(point[0]),float(point[1])]
            rejection=floor_cube_rejection(hsv,d['box'],forward,k[0],k[4])
            if forward<1.0:
                fitted=fit_floor_cube_box(d['box'],k,org,rot,point[:2])
                if fitted:
                    point[:2]=fitted[:2];d['ground_projection']=list(fitted[:2])
                    d['pose_fit_error_px']=round(fitted[2],3)
            if rejection:d['reason']=rejection;continue
            if rp:
                dx,dy=point[0]-rp[0],point[1]-rp[1]
                bx,by=math.cos(rp[2])*dx+math.sin(rp[2])*dy,-math.sin(rp[2])*dx+math.cos(rp[2])*dy
                if point_in_footprint(bx,by):d['reason']='车身范围';continue
                if self.held_model and math.hypot(bx,by)<.65:d['reason']='携带货物近车身区域';continue
            if in_delivery_area(point, extra=.1):d['reason']='已在放置区';continue
            cell=self.field.cell(point)
            if not self.field.inside(cell) or self.field.clearance[cell]<.20:
                d['reason']='投影落在墙体或地图边缘';continue
            if not self.field.line_free(org[:2],point[:2]):
                d['reason']='相机与目标之间有地图隔板';continue
            d['reason']='可抓取目标'
            result.append({'color':d['color'],'x':float(point[0]),'y':float(point[1]),'z':.025,'pixel':[u,v],
                           'box':[x,y,w,h],'source':'camera_HSV_5cm_bbox_fit' if 'pose_fit_error_px' in d else 'camera_HSV_ground_projection',
                           'pose_fit_error_px':d.get('pose_fit_error_px')})
        self.pixel_detections=candidates
        rejected={}
        for d in candidates:
            if d['reason']!='可抓取目标':rejected[d['reason']]=rejected.get(d['reason'],0)+1
        self.state['perception_note']=f'图像 / TF 正常；颜色候选 {len(candidates)}，可作业 {len(result)}' + ('；'+str(rejected) if rejected else '')
        self.state['detections']=result
        self.emit('detections',result)
        return (result,tf,image_stamp) if with_pose else result

    def llm_task(self):
        if self.task_mode == 'practice':
            return self.practice_task()
        if self.task_mode == 'exam':
            value=generate_question(BASE.parent/'competition/platform/TMSCQtest_x86_x64.bin', self.exam_config)
            self.state['question']=value['question']
            self.official_input=value
            (LOG/'exam-question.json').write_text(json.dumps(value,ensure_ascii=False,indent=2))
            self.event('已读取出题程序原题',question=value['question'],rule=self.exam_config['rule'],first=self.exam_config['first'])
        with self.lock:
            value = copy.deepcopy(self.official_input)
        if value is None:
            raise RuntimeError('尚未收到官方题目，请先 POST /api/task')
        self.state['question'] = value['question']
        task = invoke_official(value['question'], value['context'])
        self.state['task'] = task
        self.emit('task', {'question': value['question'], **task, 'source': 'exam_generator_with_model_adapter' if self.task_mode=='exam' else 'official_adapter'})
        self.command_pub.publish(String(data=json.dumps(task)))
        return task

    def set_official_task(self, value):
        if self.task_mode != 'official':
            raise ValueError('请以 YZBOT_TASK_MODE=official 启动后提交官方题目')
        if not isinstance(value, dict) or not isinstance(value.get('question'), str) or not value['question'].strip():
            raise ValueError('需要非空 question 字符串和可选 context 对象')
        context = value.get('context', {})
        if not isinstance(context, dict):
            raise ValueError('context 必须为对象')
        with self.lock:
            if self.busy:
                raise ValueError('任务运行或场景复位期间不能替换题目')
            self.official_input = {'question': value['question'], 'context': copy.deepcopy(context)}
            self.state.update(question=value['question'], task=None,
                              status='官方题目已接收；开始前请确认官方适配器已实现')

    def practice_task(self):
        rng=random.Random(int(self.state.get('layout_seed',random.SystemRandom().getrandbits(32))) ^ 0xA17E)
        red=rng.choice([2,3]); blue=5-red
        first=rng.choice(['red','blue']); rz,bz=rng.sample(list(ZONES),2)
        more='红色比蓝色多1个' if red>blue else '蓝色比红色多1个'
        order=f'先搬运{ "红色" if first=="red" else "蓝色" }，再搬运另一种颜色'
        question=f'共有5个包裹，{more}。{order}。红色放到{rz}区，蓝色放到{bz}区。求每种颜色的数量并按顺序执行。'
        self.state['question']=question
        schema={'type':'object','properties':{'red':{'type':'integer','minimum':1,'maximum':4},'blue':{'type':'integer','minimum':1,'maximum':4},
                'first':{'type':'string','enum':['red','blue']},'red_zone':{'type':'string','enum':list(ZONES)},'blue_zone':{'type':'string','enum':list(ZONES)}},
                'required':['red','blue','first','red_zone','blue_zone'],'additionalProperties':False}
        # The question program supplies order/destinations explicitly. Preserve
        # those authoritative fields with constrained decoding; counts remain solved.
        schema['properties']['first']['enum']=[first]
        schema['properties']['red_zone']['enum']=[rz]
        schema['properties']['blue_zone']['enum']=[bz]
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        def ask(messages, output_schema):
            body={'messages':messages,'temperature':0,'max_tokens':180,
                  'response_format':{'type':'json_object','schema':output_schema}}
            request=urllib.request.Request(os.environ.get('YZBOT_PRACTICE_LLM_URL','http://127.0.0.1:8081')+'/v1/chat/completions',data=json.dumps(body).encode(),headers={'Content-Type':'application/json'})
            raw=json.load(opener.open(request,timeout=45))
            return json.loads(raw['choices'][0]['message']['content']),raw
        constraint_schema=copy.deepcopy(schema)
        del constraint_schema['properties']['red'];del constraint_schema['properties']['blue']
        constraint_schema['properties'].update(total={'type':'integer'},more_color={'type':'string','enum':['red','blue']},difference={'type':'integer'})
        for field in ['first','red_zone','blue_zone']:del constraint_schema['properties'][field]
        constraint_schema['required']=['more_color','total','difference']
        english=f'Total parcels = 5. {"red - blue" if red>blue else "blue - red"} = 1. Which color has more parcels? Extract more_color, total, difference.'
        expected_constraints={'total':5,'more_color':'red' if red>blue else 'blue','difference':1,'first':first,'red_zone':rz,'blue_zone':bz}
        messages=[{'role':'system','content':'Extract equation constraints. The first color in "X - Y = 1" has more parcels. Output JSON.'},
                  {'role':'user','content':'Total = 7. red - blue = 3.'},
                  {'role':'assistant','content':'{"more_color":"red","total":7,"difference":3}'},
                  {'role':'user','content':'Total = 9. blue - red = 1.'},
                  {'role':'assistant','content':'{"more_color":"blue","total":9,"difference":1}'},
                  {'role':'user','content':english}]
        attempts=[]
        for attempt in range(3):
            constraints,raw1=ask(messages,constraint_schema)
            attempts.append({'parsed':copy.deepcopy(constraints),'raw':raw1})
            constraints.update(first=first,red_zone=rz,blue_zone=bz)
            (LOG/'last-llm-extraction-attempts.json').write_text(json.dumps(attempts,ensure_ascii=False,indent=2))
            if constraints==expected_constraints:break
            self.event('Qwen 约束校验失败，按原题重新解析',attempt=attempt+1)
            messages += [{'role':'assistant','content':json.dumps(constraints)},
                         {'role':'user','content':'The extraction is invalid. Re-read the original sentence literally. "1 more" means difference=1. Preserve authoritative fields. Original problem: '+english}]
        else:raise RuntimeError('Qwen 约束重试后仍错误，本轮停止：'+str(constraints))
        delta=constraints['difference']*(1 if constraints['more_color']=='red' else -1)
        if (constraints['total']+delta)%2:raise RuntimeError('题目没有整数解')
        tool={'red':(constraints['total']+delta)//2,'blue':(constraints['total']-delta)//2,
              'first':constraints['first'],'red_zone':constraints['red_zone'],'blue_zone':constraints['blue_zone']}
        task,raw2=ask([{'role':'system','content':'Return the exact verified tool plan as JSON. Preserve counts, first, and zones. Do not change the tool result.'},
                       {'role':'user','content':'Verified arithmetic tool result: '+json.dumps(tool)}],schema)
        (LOG/'last-llm-response.json').write_text(json.dumps({'question':question,'bilingual_input':english,'constraints':constraints,
            'arithmetic_tool':tool,'extraction_response':raw1,'plan_response':raw2},ensure_ascii=False,indent=2))
        expected={'red':red,'blue':blue,'first':first,'red_zone':rz,'blue_zone':bz}
        if task!=expected:
            raise RuntimeError(f'大模型解析未通过数量/顺序校验：{task}。本轮停止，不替换模型答案。')
        self.state['task']=task
        self.emit('task',{'question':question,**task,'source':'local_practice_generator'})
        self.command_pub.publish(String(data=json.dumps(task)))
        return task

    def start(self):
        with self.lock:
            if self.busy:return False
            if self.state['phase']!='就绪' or (self.state.get('localization')=='slam' and not self.state.get('fresh_slam_for_scene')):
                self.state['status']='请先重置场景，等待本轮导航与 SLAM 准备就绪';return False
            if self.task_mode == 'official' and self.official_input is None:
                self.state['status']='尚未收到官方题目，请先 POST /api/task';return False
            if self.held_model or self.used:
                self.state['status']='请先重置场景再开始新一轮';return False
            health=self.snapshot()
            if not health['healthy'] or self.camera_metrics.get('front',{}).get('advances',0)<5 or not self.models:
                self.state['status']='暂不能开始：'+('；'.join(health['health_issues']) or '等待连续相机帧与仿真状态');return False
            self.cancel.clear(); self.busy=True; self.started=time.monotonic();self.execution_time_factor=1.0
            self.run_id=time.strftime('%Y%m%d-%H%M%S')+'-'+str(time.time_ns()%1000000)
            (LOG/'runs'/self.run_id).mkdir(parents=True,exist_ok=True)
            self.phase_timings={};self.phase_started=time.monotonic()
            # New randomized positions invalidate earlier point-specific priors.
            self.experience=(None if self.state.get('layout_randomized') else
                             SearchExperience(self.field,BASE/'src/yzbot/bot_navigation/maps/map.yaml',
                                              BASE.parent/'data/search_experience',self.state.get('layout_mode','unknown')))
            self.experience_attempted=[]
            self.experience_misses=0
            self.state['experience_training_runs']=len(self.experience.runs) if self.experience else 0
            self.used=set(); self.trace=[];self.rviz_trace=[]
            self.grasp_attempts=0
            self.memory=ObjectMemory();self.search_memory=SearchMemory(self.field.search_points);self.search_seen=self.search_memory.seen;self.search_attempted={}
            self.perception_paused=False
            self.last_observed_stamp=None;self.observation_enabled=True
            self.state.update(running=True,picked=0,placed=0,result=[],events=[],task=None)
            threading.Thread(target=self.run,daemon=True).start()
            return True

    def stop(self):
        self.cancel.set();self.halt_motion()

    def observation_session(self, enabled):
        """Lease motion for a visual survey; this is never an exam result."""
        with self.lock:
            if enabled:
                if self.busy:return False
                self.cancel.clear();self.busy=True;self.started=None
                self.observation_mode=True;self.observation_started=time.monotonic()
                self.run_id='survey-'+time.strftime('%Y%m%d-%H%M%S')
                (LOG/'runs'/self.run_id).mkdir(parents=True,exist_ok=True)
                self.observation_enabled=True;self.perception_paused=False
                self.state['fresh_slam_for_scene']=False
                self.memory=ObjectMemory();self.saved_mode=self.state['mode']
                self.state.update(mode='视觉经验采集（非考试）',phase='采集视觉经验',
                                  status='按地图观察点采集；不抓取、不计比赛成绩',running=False)
            elif self.observation_mode:
                self.observation_enabled=False;self.observation_mode=False;self.busy=False
                self.state.update(mode=self.saved_mode,phase='停止' if self.cancel.is_set() else '就绪',
                                  status='视觉经验采集已结束，正式作业前重新复位')
            else:return False
        return True

    def reset_scene(self, seed=None, easy=False, fixed=False):
        with self.lock:
            if self.busy:return False
            self.busy=True;self.cancel.clear();self.started=None
            self.run_id=None;self.phase_timings={}
            self.observation_enabled=False
            self.state['fresh_slam_for_scene']=False
            self.state.update(phase='准备场景',status='正在重置场景')
            def reset_worker():
                try:
                    self.stop_vel.publish(Twist());self.vel.publish(Twist())
                    # Gazebo constraints outlive a controller restart. Clear all
                    # of this robot's cargo constraints before moving entities,
                    # even when the new Python process has no held_model state.
                    cargo={name for name in self.models if name.startswith(('red_cube','blue_cube'))}
                    if self.held_model:cargo.add(self.held_model)
                    for name in sorted(cargo):
                        self.call(self.detach,DetachLink.Request(model1_name='six_arm',link1_name='link6',model2_name=name,link2_name='link'))
                    self.held_model=None
                    msg=JointTrajectory(joint_names=JOINTS,points=[JointTrajectoryPoint(positions=[0.]*6,time_from_start=Duration(sec=2))])
                    self.arm_pub.publish(msg);self.grip(False);time.sleep(2)
                    # Explicitly reposition only required entities. Gazebo world reset
                    # also resets plugin internals and left existing Nav2 TF listeners
                    # with a stale rolling-costmap centre during repeated trials.
                    state=EntityState(name='six_arm',reference_frame='world')
                    state.pose.orientation.w=1.
                    if not self.call(self.set_entity,SetEntityState.Request(state=state)).success:
                        raise RuntimeError('机器人出生点复位失败')
                    for _ in range(20):
                        self.stop_vel.publish(Twist());self.vel.publish(Twist());time.sleep(.05)
                    default_seed=4 if fixed else 20261019 if easy else None
                    layout_seed=(default_seed if default_seed is not None else random.SystemRandom().getrandbits(32)) if seed is None else int(seed)
                    layout=(self.field.easy_visible_layout(layout_seed) if easy
                            else self.field.scattered_layout(layout_seed))
                    for item in layout['layout']:
                        entity=EntityState(name=item['model'],reference_frame='world')
                        entity.pose.position.x=item['x'];entity.pose.position.y=item['y'];entity.pose.position.z=.025
                        entity.pose.orientation.w=1.
                        if not self.call(self.set_entity,SetEntityState.Request(state=entity)).success:
                            raise RuntimeError('随机摆放失败: '+entity.name)
                    self.state['layout_seed']=layout_seed
                    stable_layout=easy or fixed
                    self.state['layout_randomized']=not stable_layout
                    self.state['layout_mode']=('fixed_visible' if easy else 'fixed_distributed') if stable_layout else 'distributed_office'
                    self.state['layout_sector_count']=layout['sector_count']
                    (LOG/'random-layout.json').write_text(json.dumps(layout,ensure_ascii=False,indent=2))
                    time.sleep(.5)
                    actual=self.models.get('six_arm')
                    if actual is None or math.hypot(actual.position.x,actual.position.y)>.1:
                        raise RuntimeError('复位位置核验未通过')
                    # Practice uses Gazebo world odometry and a static map alignment.
                    ready_status='场景已就绪，可开始考试'
                    self.state.update(status='场景已复位，正在准备本轮导航与 SLAM',question='',task=None,picked=0,placed=0,result=[],elapsed=0,remaining=300,running=False)
                    self.memory=ObjectMemory();self.active_target_position=None;self.perception_paused=False
                    self.grasp_attempts=0
                    self.publish_placed_cargo()
                    time.sleep(.3)
                    # A world-entity reset does not clear slam_toolbox's pose
                    # graph. Restart the project navigation group before a new
                    # round so historical occupancy cannot change its routes.
                    if self.state.get('localization')=='slam':
                        self.navigation_footprints={}
                        project=FilePath(os.environ.get('YZBOT_PROJECT',str(BASE.parent)))
                        result=subprocess.run(['bash',str(project/'native/restart-navigation.sh')],
                                              stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                                              text=True,timeout=85)
                        (LOG/'reset-navigation-latest.log').write_text(result.stdout)
                        if result.returncode:raise RuntimeError('本轮导航与 SLAM 初始化失败：'+result.stdout[-500:])
                        self.state['fresh_slam_for_scene']=True
                    for client in self.clear_costmaps:
                        self.call(client,ClearEntireCostmap.Request(),5)
                    deadline=time.monotonic()+8
                    while time.monotonic()<deadline:
                        health=self.navigation_health()
                        if len(health)==2 and all(v['pose_age']<1.5 and v['receive_age']<1.5 for v in health.values()):break
                        self.check();time.sleep(.1)
                    else:raise RuntimeError('新一轮导航位姿未持续刷新，场景未标记就绪')
                    self.used=set();self.trace=[];self.rviz_trace=[];self.memory=ObjectMemory();self.search_memory=SearchMemory(self.field.search_points);self.search_seen=self.search_memory.seen;self.search_attempted={}
                    self.navigation_metrics=[];self.navigation_feedback={}
                    self.state.update(phase='就绪',status=ready_status)
                    self.event('场景重置完成，机器人与导航就绪')
                except Exception as e:self.update('重置失败',str(e))
                finally:self.busy=False
            threading.Thread(target=reset_worker,daemon=True).start();return True

    def find_target(self, remaining, zones, number):
        color=remaining[0]
        exhausted=False
        while True:
            self.check()
            rp=self.robot_pose()
            candidates=self.memory.available()
            budget=max(0.,300.-self.elapsed()-3.)
            route=choose_observed_deliveries(self.field,rp,candidates,remaining,zones,number,time_budget=budget,
                                  time_scale=self.execution_time_factor)
            if route:
                self.state['known_plan']=[{'id':r['target']['id'],'color':r['target']['color'],
                                          'x':r['target']['x'],'y':r['target']['y'],
                                          'seconds':r['estimated_seconds']} for r in route]
                selected=route[0]
                if selected.get('deadline_fallback'):
                    self.event('剩余预算不足，优先尝试已确认目标，避免末段重复搜索',
                               target_id=selected['target']['id'],estimated_seconds=selected['estimated_seconds'],
                               budget_seconds=round(budget,1))
                self.event('按可通行路径选择搬运目标',target_id=selected['target']['id'],
                           known_plan=self.state['known_plan'],estimated_seconds=selected['known_plan_seconds'],
                           budget_seconds=round(budget,1),deadline_slack_seconds=round(budget-selected['known_plan_seconds'],1),
                           execution_time_factor=round(self.execution_time_factor,2))
                return selected
            camera_tf=self.buffer.lookup_transform('base_control','camera_link',Time()).transform
            q=camera_tf.rotation
            camera_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            inspected=self.experience_attempted+[(t['x'],t['y']) for t in self.memory.snapshot()
                                                 if t['status'] in ('picked','delivered')]
            prior=(self.experience.observation(rp,color,inspected,zones[color])
                   if self.experience and self.experience_misses<2 else None)
            if prior:
                pose=(*prior['pose'][:2],angle(prior['pose'][2]-camera_yaw))
                self.update('经验引导复核',f"前往历史相机观察区域复核{color}目标；本轮图像确认后才抓取")
                self.event('跨轮视觉经验仅用于选择观察点',confidence=prior['confidence'],
                           observation_pose=list(pose),training_runs=len(self.experience.runs))
                try:
                    known_color=any(t['color']==color for t in candidates)
                    reached=self.go(*pose,look_for=None if known_color else color)
                    if reached:
                        self.experience_attempted.append(prior['centre'])
                        time.sleep(.35)
                        if not self.memory.available(color):self.experience_misses+=1
                except Exception as error:
                    self.check();self.experience_attempted.append(prior['centre'])
                    self.experience_misses+=1
                    self.event('经验区域未能复核，继续视觉搜索',reason=str(error))
                continue
            optical_pose=(*rp[:2],angle(rp[2]+camera_yaw))
            view=self.field.next_view(optical_pose,self.search_seen,self.search_attempted,delivery=zones[color],
                                      directional=not self.state['placed'])
            if view is None:
                delay=self.memory.retry_delay(color)
                if delay>0:
                    self.update('等待重试',f'候选目标冷却中，{delay:.1f} 秒后重新评估')
                    retry_end=time.monotonic()+min(2.,delay)
                    while time.monotonic()<retry_end:
                        self.check();time.sleep(.1)
                    continue
                if exhausted:raise RuntimeError('可达观察点已检查，尚未找到足够目标；保留空区记录')
                self.search_attempted={};exhausted=True
                self.event('保留已确认空区，仅重新检查未看清的区域');continue
            _,idx,pose,gain,cost=view
            pose=(*pose[:2],angle(pose[2]-camera_yaw))
            self.search_attempted[idx]=self.search_attempted.get(idx,0)+1
            self.state['search_goal']={'x':pose[0],'y':pose[1],'yaw':pose[2],'new_cells':gain}
            label='红色' if color=='red' else '蓝色'
            self.update('搜索隔间',f'寻找{label}目标，前往观察点 ({pose[0]:.1f}, {pose[1]:.1f})')
            known_color=any(t['color']==color for t in candidates)
            try:
                # When known targets cannot fit the remaining time estimate,
                # keep visiting new views for nearer alternatives. Otherwise
                # go() immediately cancels each new goal on the known target.
                reached=self.go(*pose,look_for=None if known_color else color)
                if reached:
                    self.update('定向识别',f'当前机位识别{label}目标；未发现就前往下一个观察点')
                    # The camera observer runs continuously on fresh frames. Give
                    # it a short settle window, then choose another view. Never
                    # issue a Spin action here: scanning in place caused long turns.
                    deadline=time.monotonic()+.25
                    while time.monotonic()<deadline:
                        self.check()
                        if any(self.field.approaches(t) for t in self.memory.available(color)):
                            break
                        time.sleep(.05)
            except Exception as e:
                self.check()
                self.event('观察点暂时不可达，重新选择',reason=str(e))

    def approach_target(self, selection):
        self.grasp_preflight=None
        target=selection['target']
        self.update('直达目标抓取位',f"相机已发现 {target['color']} #{target['id']}，直达近处抓取位")
        self.active_target_position=(target['x'],target['y'])
        self.publish_placed_cargo();time.sleep(.25)
        for client in self.clear_costmaps:self.call(client,ClearEntireCostmap.Request(),5)
        self.settle_base()
        # Once the camera has found the cube, prefer the robot's current side
        # and one straight run to a safe arm-reach stop point. The inflated
        # static map can reject a clear center lane near partition corners, so
        # fresh lidar validates that case after the single heading adjustment.
        pregrasp=selection.get('pregrasp',selection['view'])
        stage=(target['x']-.50*math.cos(pregrasp[2]),
               target['y']-.50*math.sin(pregrasp[2]),pregrasp[2])
        if self.field.safe(stage) and self.field.direct_base_path_clear(stage,pregrasp):
            pregrasp=stage
        pose=self.robot_pose()
        direct=False
        if pose:
            tx,ty=float(target['x']),float(target['y'])
            distance=math.hypot(tx-pose[0],ty-pose[1])
            if distance>.30 and self.field.line_free(pose[:2],(tx,ty)):
                yaw=math.atan2(ty-pose[1],tx-pose[0])
                gap=min(.50,distance)
                candidate=(tx-gap*math.cos(yaw),ty-gap*math.sin(yaw),yaw)
                if self.field.safe(candidate):
                    pregrasp=candidate
                    selection['grasp']=(tx-.24*math.cos(yaw),ty-.24*math.sin(yaw),yaw)
                    selection['pregrasp']=candidate
                    selection['view']=candidate;selection['direct_from_current']=True
                    direct=True
            if not direct:
                direct=bool(selection.get('direct_from_current')
                            and self.field.direct_base_path_clear(pose,pregrasp))
        if direct:
            yaw_error=angle(pregrasp[2]-pose[2])
            if abs(yaw_error)>.06:
                self.align(pose[0],pose[1],pregrasp[2],position_tolerance=.025,yaw_tolerance=.05)
            pose=self.robot_pose()
            dx,dy=pregrasp[0]-pose[0],pregrasp[1]-pose[1]
            forward=dx*math.cos(pose[2])+dy*math.sin(pose[2])
            lateral=-dx*math.sin(pose[2])+dy*math.cos(pose[2])
            if not (.08<forward<6.0 and abs(lateral)<.12):direct=False
            if direct and not self.field.direct_base_path_clear(pose,pregrasp):
                deadline=time.monotonic()+.5
                while time.monotonic()<deadline and time.monotonic()-self.latest_scan_time>.5:
                    self.check();time.sleep(.05)
                if not self.lidar_lane_clear(forward):direct=False
        if direct:
            speed=min(1.2,max(.5,forward)) if self.profiled_straight else .5
            command=DriveOnHeading.Goal(target=Point(x=float(forward)),speed=float(speed),
                                        time_allowance=Duration(sec=12))
            handle=self.wait(self.drive_action.send_goal_async(command),5)
            if not handle.accepted:raise RuntimeError('Nav2 拒绝目标直线靠近')
            self.nav_handle=handle
            pending=handle.get_result_async()
            try:outcome=self.wait(pending,15)
            finally:
                if not pending.done():handle.cancel_goal_async()
                self.nav_handle=None
            after=self.robot_pose()
            remaining=math.dist(after[:2],pregrasp[:2]) if after else math.inf
            if outcome.status==4 and remaining<=.18:
                self.event('朝相机确认目标直线驶向抓取点',distance=round(forward,3),
                           remaining=round(remaining,3),lateral_error=round(lateral,3),
                           heading_correction=round(yaw_error,3))
            else:
                self.event('直线受障碍截停，继续导航到抓取点',remaining=round(remaining,3),
                           action_status=outcome.status)
                direct=False
        if not direct:
            self.update('目标前方需绕障', '通道被隔板或障碍挡住，沿安全路径到达抓取侧')
            self.go(*pregrasp,align_tolerance=.10,align_yaw_tolerance=.06)
        self.settle_base()
        time.sleep(.10)
        detections=[d for d in self.vision() if d['color']==target['color']
                    and math.hypot(d['x']-target['x'],d['y']-target['y'])<.30]
        # Wait briefly for another camera frame at the same pose. A second
        # nearby navigation goal often caused an unnecessary turn before grasp.
        if not detections:
            self.event('抓取位等待相机新帧',target_id=target['id'])
            deadline=time.monotonic()+.30
            while time.monotonic()<deadline and not detections:
                self.check()
                time.sleep(.05)
                detections=[d for d in self.vision() if d['color']==target['color']
                            and math.hypot(d['x']-target['x'],d['y']-target['y'])<.30]
        if not detections:
            rejected=[{'box':v.get('box'),'reason':v.get('reason')}
                      for v in self.pixel_detections if v.get('color')==target['color']]
            self.event('抵近图像未通过识别筛选',target_id=target['id'],
                       color=target['color'],pixel_candidates=rejected,
                       camera_age=round(max(0.,time.monotonic()-self.image_time),3))
            refinements=[]
            for candidate in self.pixel_detections:
                point=confirmed_near_target(candidate,target)
                if point is not None:refinements.append((candidate,point))
            if refinements:
                candidate,(x,y)=min(refinements,key=lambda item:math.hypot(
                    item[1][0]-target['x'],item[1][1]-target['y']))
                detections=[{'color':target['color'],'x':x,'y':y,'z':.025,
                             'pixel':candidate.get('box'),'source':'confirmed_near_camera'}]
                self.event('近景图像校正已确认目标位置',target_id=target['id'],
                           box=candidate.get('box'),ground_projection=[x,y])
        if detections:
            d=min(detections,key=lambda d:math.hypot(d['x']-target['x'],d['y']-target['y']))
        elif int(target.get('hits',0))>=2:
            # The camera can lose a cube at very close range when it falls
            # below the image crop. Keep the multi-frame camera localization
            # acquired before arrival instead of backing the robot away.
            d={'color':target['color'],'x':float(target['x']),'y':float(target['y']),
               'z':.025,'source':'confirmed_camera_memory'}
            self.event('近处视野暂时遮挡，沿用已确认相机定位',target_id=target['id'],
                       camera_hits=int(target['hits']))
        else:
            raise RuntimeError('抵近后目标定位不足，未执行盲抓')
        # Use synchronized close-range frames to place the fingers accurately.
        samples=[];stamp=None
        for _ in range(5):
            self.check()
            values,_,frame_stamp=self.vision(with_pose=True)
            key=(frame_stamp.sec,frame_stamp.nanosec)
            nearby=[v for v in values if v['color']==target['color']
                    and math.hypot(v['x']-d['x'],v['y']-d['y'])<.12]
            if key!=stamp and nearby:
                samples.append(min(nearby,key=lambda v:math.hypot(v['x']-d['x'],v['y']-d['y'])))
                stamp=key
            if len(samples)>=2:break
            time.sleep(.08)
        if len(samples)>=2:
            d=dict(samples[-1],x=float(np.median([v['x'] for v in samples])),
                   y=float(np.median([v['y'] for v in samples])))
        elif d.get('source')!='confirmed_camera_memory':
            # A single fresh frame is enough to refine a target already
            # confirmed across multiple frames; do not restart the search.
            self.event('近距离复核帧不足，保留已确认目标位置',target_id=target['id'],
                       fresh_frames=len(samples))
        pose=self.robot_pose()
        distance=math.hypot(d['x']-pose[0],d['y']-pose[1])
        if distance>.28:
            if distance>.85:raise RuntimeError('抵近后的视觉距离异常，取消最后推进')
            if d.get('source')=='confirmed_camera_memory' and not samples:
                raise RuntimeError('没有抵近图像复核，取消最后推进')
            yaw=math.atan2(d['y']-pose[1],d['x']-pose[0])
            self.align(pose[0],pose[1],yaw,position_tolerance=.025,yaw_tolerance=.04)
            forward=distance-.24
            command=DriveOnHeading.Goal(target=Point(x=float(forward)),speed=.22,
                                        time_allowance=Duration(sec=8))
            handle=self.wait(self.drive_action.send_goal_async(command),5)
            if not handle.accepted:raise RuntimeError('最后抵近被碰撞检查拒绝')
            self.nav_handle=handle;pending=handle.get_result_async()
            try:outcome=self.wait(pending,10)
            finally:
                if not pending.done():handle.cancel_goal_async()
                self.nav_handle=None
            if outcome.status!=4:raise RuntimeError('最后抵近未完成，保留目标重新观察')
            self.settle_base()
            self.event('抵近图像复核后限速直线夹取',distance=round(forward,3),stop_gap_m=.24)
        else:
            yaw=selection['grasp'][2]
        # Stop the base 22 cm before the target: this keeps the cube in view,
        # clears the chassis, and leaves the last reach to the arm.
        grasp=(d['x']-.24*math.cos(yaw),d['y']-.24*math.sin(yaw),yaw)
        if not self.field.safe(grasp):raise RuntimeError('精对准抓取位姿靠墙，换其他目标')
        # The chassis has already reached the single image-guided stop pose.
        # Do not send another base goal after reacquiring the cube.
        self.event('保持安全间距，机械臂直接抓取',base_target_gap_m=.24)
        # Simulator identity is resolved only here, after image-guided arrival.
        # It supplies attachment identity/contact verification, never a search goal.
        available=[(name,p) for name,p in self.models.items()
                   if name.startswith(target['color']+'_cube') and name not in self.used]
        if not available:raise RuntimeError('没有可用于仿真夹持核验的实体')
        name,p=min(available,key=lambda item:math.hypot(item[1].position.x-d['x'],item[1].position.y-d['y']))
        error=math.hypot(p.position.x-d['x'],p.position.y-d['y'])
        if error>.12:raise RuntimeError(f'相机定位与近处实体不一致 {error:.3f}m')
        # A map-safe approach can still put the arm outside its reachable
        # workspace. Check both descent poses before committing to the grasp;
        # if this side is unreachable, try the nearest safe cardinal side while
        # keeping the same visually confirmed cube.
        try:
            grip_yaw=square_gripper_yaw(self.robot_pose()[2])
            hover=self.solve_ik(d['x'],d['y'],.18,world_yaw=grip_yaw)
            descent=self.solve_ik(d['x'],d['y'],.10,world_yaw=grip_yaw)
        except RuntimeError as first_ik_error:
            pose=self.robot_pose()
            if not pose:raise
            routes=self.field.distances(pose)
            alternatives=[]
            for candidate in self.field.approaches({'x':d['x'],'y':d['y']}):
                grasp=candidate['grasp']
                if math.dist(pose[:2],grasp[:2])<.10:continue
                cost=self.field.distance(routes,grasp)
                if math.isfinite(cost):alternatives.append((cost,candidate))
            found=False
            for _,candidate in sorted(alternatives,key=lambda item:item[0]):
                try:
                    self.event('当前夹取侧机械臂不可达，切换最近安全侧',
                               target_id=target['id'],reason=str(first_ik_error),
                               candidate=candidate['grasp'])
                    self.go(*candidate['grasp'])
                    self.settle_base()
                    # Reconfirm the same cube after changing sides when a fresh
                    # image is available; otherwise retain its confirmed pose.
                    fresh=[v for v in self.vision() if v['color']==target['color']
                           and math.hypot(v['x']-d['x'],v['y']-d['y'])<.30]
                    if fresh:
                        d=min(fresh,key=lambda v:math.hypot(v['x']-d['x'],v['y']-d['y']))
                        p=self.models[name].position
                        if math.hypot(p.x-d['x'],p.y-d['y'])>.12:
                            continue
                    grip_yaw=square_gripper_yaw(self.robot_pose()[2])
                    hover=self.solve_ik(d['x'],d['y'],.18,world_yaw=grip_yaw)
                    descent=self.solve_ik(d['x'],d['y'],.10,world_yaw=grip_yaw)
                    selection['grasp']=candidate['grasp']
                    selection['pregrasp']=candidate['pregrasp']
                    selection['view']=candidate['view']
                    found=True
                    break
                except RuntimeError:
                    continue
            if not found:
                raise RuntimeError(f'所有安全夹取侧均无可行 IK: {first_ik_error}')
        self.grasp_preflight={'base':self.robot_pose(),'target':(d['x'],d['y']),
                             'world_yaw':grip_yaw,'hover':hover,'descent':descent,
                             'time':time.monotonic()}
        self.event('相机抵近确认目标',target_id=target['id'],model=name,detection=d,
                   grasp_pose=selection['grasp'])
        return name,d

    def pick_up(self, name, d, target_id):
        self.perception_paused=True
        if self.state.get('gripper')!='张开':self.grip(False)
        self.update('抓取',f'机械臂正在抓取 {name}')
        self.grasp_attempts+=1
        # Check both arm poses before beginning the descent. A failed IK check
        # must not trigger a blind base charge toward the cube.
        grip_yaw=square_gripper_yaw(self.robot_pose()[2])
        if grasp_preflight_valid(self.grasp_preflight,self.robot_pose(),
                                 (d['x'],d['y']),grip_yaw,time.monotonic()):
            hover,descent=self.grasp_preflight['hover'],self.grasp_preflight['descent']
            self.event('复用当前停稳姿态的抓取预检，连续执行')
        else:
            hover=self.solve_ik(d['x'],d['y'],.18,world_yaw=grip_yaw)
            descent=self.solve_ik(d['x'],d['y'],.10,world_yaw=grip_yaw)
        self.grasp_preflight=None
        self.event('手腕按方块边缘对齐，车身无需额外转向',world_gripper_yaw=round(grip_yaw,3))
        self.arm(hover)
        self.arm(descent,speed=.55)
        p=self.models[name].position;hand=self.links.get('six_arm::link6')
        if hand is None:raise RuntimeError('缺少真实末端 link6 位姿')
        distance=math.dist([p.x,p.y,p.z],[hand.position.x,hand.position.y,hand.position.z])
        if distance>.105:raise RuntimeError(f'末端距货物 {distance:.3f}m，禁止远距离附着')
        self.grip(True)
        req=AttachLink.Request(model1_name='six_arm',link1_name='link6',model2_name=name,link2_name='link')
        r=self.call(self.attach,req)
        if not r.success:raise RuntimeError(r.message)
        self.held_model=name;self.state['gripper']='已抓住 '+name
        # A long horizontal reach that is valid at floor height may be outside
        # the workspace when lifted vertically. Lift and retract together.
        rp=self.robot_pose();bx=rp[0]-.14*math.cos(rp[2]);by=rp[1]-.14*math.sin(rp[2])
        reach=math.hypot(d['x']-bx,d['y']-by)
        scale=min(1.,.34/max(.001,reach))
        self.arm_xyz(bx+(d['x']-bx)*scale,by+(d['y']-by)*scale,.22,world_yaw=grip_yaw)
        lift_deadline=time.monotonic()+.45
        while self.models[name].position.z<.12 and time.monotonic()<lift_deadline:
            self.check();time.sleep(.05)
        if self.models[name].position.z<.12:raise RuntimeError('货物未随机械臂抬升')
        self.state['picked']+=1;self.used.add(name)
        self.memory.mark(target_id,'picked')
        self.active_target_position=None
        self.search_memory.collected((d['x'],d['y']))
        self.event('抓取并抬升确认',model=name,contact_distance=distance,z=self.models[name].position.z)

    def run(self):
        try:
            self.update('大模型解题','正在运行出题程序并调用模型解题' if self.task_mode=='exam' else ('等待官方模型适配器解题' if self.task_mode=='official' else '随机题目已生成，等待本地 Qwen 解题'))
            task=self.llm_task()
            self.check()
            self.event('大模型已输出并通过校验',answer=task)
            try:
                values=self.call(self.behavior_parameters,GetParameters.Request(names=[
                    'drive_on_heading.acceleration_limit','drive_on_heading.deceleration_limit']),3).values
                self.profiled_straight=(len(values)==2 and values[0].type==3 and values[1].type==3
                                        and values[0].double_value>0 and values[1].double_value<0)
            except Exception:self.profiled_straight=False
            self.event('直线抵近制动能力核验',acceleration_profile=self.profiled_straight,
                       maximum_approach_mps=1.2 if self.profiled_straight else .5)
            self.update('离开出生点','先后退 0.40 m，为安全搜索留出空间')
            if not self.backup_action.wait_for_server(timeout_sec=3):raise RuntimeError('后退行为不可用')
            command=BackUp.Goal(target=Point(x=.40),speed=.25,time_allowance=Duration(sec=8))
            self.nav_handle=self.wait(self.backup_action.send_goal_async(command),5)
            if not self.nav_handle.accepted:raise RuntimeError('Nav2 拒绝后退')
            outcome=self.wait(self.nav_handle.get_result_async(),10);self.nav_handle=None
            if outcome.status!=4:raise RuntimeError('安全后退未完成')
            colors=[task['first'],'blue' if task['first']=='red' else 'red']
            sequence=[c for c in colors for _ in range(task[c])]
            zones={c:ZONES[task[c+'_zone']] for c in colors}
            self.state['strategy']='map_search_camera_memory_exact_known_assignment'
            for number,color in enumerate(sequence):
                while True:
                    self.check()
                    selection=self.find_target(sequence[number:],zones,number)
                    delivery_started=self.elapsed()
                    try:
                        name,d=self.approach_target(selection)
                        self.pick_up(name,d,selection['target']['id'])
                        break
                    except Exception as e:
                        self.check()
                        if self.held_model is not None:raise
                        self.active_target_position=None
                        # Retract safely and re-observe after a missed contact.
                        self.grip(False)
                        rp=self.robot_pose()
                        self.arm_xyz(rp[0]+.10*math.cos(rp[2]),rp[1]+.10*math.sin(rp[2]),.43,speed=.6)
                        self.perception_paused=False
                        self.memory.fail(selection['target']['id'])
                        self.event('抓取未确认，撤臂后重新观察',target_id=selection['target']['id'],reason=str(e))
                rp=self.robot_pose()
                self.arm_xyz(rp[0]+.10*math.cos(rp[2]),rp[1]+.10*math.sin(rp[2]),.43)
                self.perception_paused=False
                zone=task[color+'_zone'];zx,zy=ZONES[zone]
                occupied=[item.get('final_position',item['position'])[:2]
                          for item in self.state['result'] if item['zone']==zone]
                field=self.field.distances(rp)
                options=[]
                for landing,base in delivery_region_approaches((zx,zy),occupied):
                    if not self.field.safe(base) or not self.field.line_free(base,landing):continue
                    if not usable_drop_candidates(self.field,(zx,zy),landing,base,occupied):continue
                    travel=self.field.distance(field,base)
                    bearing=math.atan2(base[1]-rp[1],base[0]-rp[0])
                    turns=abs(angle(bearing-rp[2]))+abs(angle(base[2]-bearing))
                    cost=travel/float(SCENE.get('cruise_speed_mps',1.5))+.35*turns+math.dist(landing,(zx,zy))
                    if math.isfinite(cost):options.append((cost,landing,base))
                if not options:raise RuntimeError('放置区没有避开已放货物的可达停靠侧')
                _,(px,py),drop_base=min(options,key=lambda item:item[0])
                self.event('联合选择空闲落点与停靠侧',zone=zone,landing=[px,py],base=list(drop_base))
                self.update('搬运',f'携带 {name} 前往 {zone} 区')
                self.go(*drop_base,align_tolerance=.10,align_yaw_tolerance=math.pi,
                        arrive_region=((zx,zy),(px,py),occupied))
                self.settle_base()
                self.update('放置',f'正在把 {name} 放入 {zone} 区')
                self.perception_paused=True
                # Coarse Nav2 arrival is normally enough. Check both arm poses
                # first and refine the base only if the target is out of reach.
                candidates=usable_drop_candidates(self.field,(zx,zy),(px,py),self.robot_pose(),occupied)
                release=None
                # Try a bounded number of geometry-safe interior points. A
                # feasible landing inside the zone needs no exact slot/yaw.
                for candidate in candidates[:4]:
                    try:
                        release=self.release_ik(*candidate,max_seeds=1);px,py=candidate;break
                    except RuntimeError:continue
                if release is not None:
                    hover_joints,release_joints=release
                    self.event('沿用到区姿态直接放置',zone=zone,landing=[px,py])
                else:
                    self.event('仅为安全可达性调整放置姿态',reason='当前姿态没有通过 IK 核验的空闲区内落点')
                    # Nav2 checks both translation and orientation. Do not use
                    # the fine-position servo to traverse a distant docking pose.
                    pose=self.robot_pose()
                    if math.dist(pose[:2],drop_base[:2])>.12 or abs(angle(pose[2]-drop_base[2]))>.15:
                        self.go(*drop_base,align_tolerance=.10,align_yaw_tolerance=.10)
                    self.settle_base()
                    self.refine_docking_distance(drop_base)
                    release=None
                    for candidate in usable_drop_candidates(self.field,(zx,zy),(px,py),self.robot_pose(),occupied)[:4]:
                        try:
                            release=self.release_ik(*candidate,max_seeds=2);px,py=candidate;break
                        except RuntimeError:continue
                    if release is None:raise RuntimeError('安全停靠后仍无通过 IK 核验且可以退出的区内落点')
                    hover_joints,release_joints=release
                self.arm(hover_joints,speed=.9);self.arm(release_joints,speed=.65)
                req=DetachLink.Request(model1_name='six_arm',link1_name='link6',model2_name=name,link2_name='link')
                r=self.call(self.detach,req)
                if not r.success:raise RuntimeError(r.message)
                self.held_model=None;self.grip(False)
                settle_deadline=time.monotonic()+.55;stable=0;last_position=None
                while time.monotonic()<settle_deadline:
                    self.check();p=self.models[name].position
                    position=(p.x,p.y,p.z)
                    stable=stable+1 if p.z<.06 and last_position is not None and math.dist(position,last_position)<.006 else 0
                    if stable>=2:break
                    last_position=position;time.sleep(.05)
                p=self.models[name].position
                valid=abs(p.x-zx)<.5 and abs(p.y-zy)<.25 and p.z<.16
                if not valid:raise RuntimeError(f'货物未落在指定 {zone} 区')
                self.state['placed']+=1;self.memory.mark(selection['target']['id'],'delivered')
                result={'model':name,'zone':zone,'position':[p.x,p.y,p.z],'verified':True}
                self.state['result'].append(result);self.event('放置位置确认',**result)
                # Return to the already validated hover pose. Raising at the
                # same XY to an arbitrary height can be outside joint limits.
                self.arm(hover_joints,speed=.9)
                rp=self.robot_pose()
                self.arm_xyz(rp[0]+.10*math.cos(rp[2]),rp[1]+.10*math.sin(rp[2]),.43)
                # Region placement can leave cargo beside or behind the base;
                # depart away from its actual landing, always collision checked.
                self.retreat_after_drop((p.x,p.y))
                self.perception_paused=False
                actual=max(.1,self.elapsed()-delivery_started)
                estimate=max(.1,float(selection['estimated_seconds']))
                observed=max(1.0,min(2.25,actual/estimate))
                self.execution_time_factor=max(1.0,min(2.25,.35*self.execution_time_factor+.65*observed))
                self.event('按实测取放耗时校正路线估时',actual_seconds=round(actual,1),
                           previous_estimate_seconds=round(estimate,1),
                           observed_ratio=round(observed,2),next_time_factor=round(self.execution_time_factor,2))
            invalid=[]
            for item in self.state['result']:
                p=self.models[item['model']].position;zx,zy=ZONES[item['zone']]
                item['final_position']=[p.x,p.y,p.z]
                item['final_verified']=abs(p.x-zx)<.5 and abs(p.y-zy)<.25 and p.z<.16
                if not item['final_verified']:invalid.append(item['model'])
            if invalid:
                self.state['placed']=sum(item['final_verified'] for item in self.state['result'])
                raise RuntimeError('结束核验发现货物离开放置区：'+', '.join(invalid))
            self.update('完成','5 个分散货物已按指令完成搜索、抓取和放置，终检通过')
        except Exception as e:
            self.update('停止' if self.cancel.is_set() else '未完成',str(e))
        finally:
            self.halt_motion();self.observation_enabled=False
            # Recheck earlier deliveries even when a timeout or another action
            # interrupts the round; attempted placements are not final credit.
            for item in self.state['result']:
                pose=self.models.get(item['model']);zx,zy=ZONES[item['zone']]
                if pose is None:item['final_verified']=False;continue
                p=pose.position
                item['final_position']=[p.x,p.y,p.z]
                item['final_verified']=abs(p.x-zx)<.5 and abs(p.y-zy)<.25 and p.z<.16
            self.state['placed']=sum(bool(item.get('final_verified')) for item in self.state['result'])
            self.state.update(running=False,elapsed=round(self.elapsed(),1),remaining=max(0,round(300-self.elapsed(),1)))
            self.busy=False
            final=self.snapshot()
            if self.experience and self.run_id:
                self.experience.learn(final.get('known_targets',[]),self.run_id)
            (LOG/'competition-result.json').write_text(json.dumps(final,ensure_ascii=False,indent=2))
            if self.run_id:
                directory=LOG/'runs'/self.run_id
                (directory/'result.json').write_text(json.dumps(final,ensure_ascii=False,indent=2))
                review=review_markdown(final,self.phase_timings)
                (directory/'review.txt').write_text(review)
                (LOG/'mission-review-latest.txt').write_text(review)


class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass

    def reply(self,body,kind='application/json',code=200):
        self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store')
        self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)

    def do_GET(self):
        path=self.path.split('?')[0]
        if path=='/api/state':return self.reply(json.dumps(app.snapshot(),ensure_ascii=False).encode())
        if path in ('/api/image', '/api/image/raw'):
            if app.image is None:return self.reply(b'',code=503)
            if path=='/api/image/raw' and app.camera_jpeg.get('front'):
                return self.reply(app.camera_jpeg['front'],'image/jpeg')
            image=app.image.copy()
            for d in app.pixel_detections if path=='/api/image' else []:
                if d.get('reason') != '可抓取目标':continue
                x,y,w,h=d['box'];cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
            return self.reply(cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,75])[1].tobytes(),'image/jpeg')
        if path in ['/api/camera/overview', '/api/camera/oblique']:
            view=path.rsplit('/',1)[-1]
            data=app.camera_jpeg.get(view)
            if data is None:return self.reply(b'',kind='image/jpeg',code=503)
            return self.reply(data,'image/jpeg')
        if path=='/api/slam-map':
            try:
                return self.reply(json.dumps(app.map_view(),ensure_ascii=False).encode())
            except Exception as error:
                return self.reply(json.dumps({'error':str(error)},ensure_ascii=False).encode(),code=503)
        if path=='/api/map':
            raw=app.field.display_map
            return self.reply(cv2.imencode('.png',raw)[1].tobytes(),'image/png')
        if path=='/fpv':return self.reply(b'<!doctype html><meta charset=utf-8><title>Robot FPV</title><style>body{margin:0;background:#111;color:white;font:18px sans-serif}img{width:100vw;height:90vh;object-fit:contain}p{margin:6px}</style><img id=i><p id=s></p><script>setInterval(()=>i.src="/api/image?t="+Date.now(),180);setInterval(async()=>{let a=await(await fetch("/api/state")).json();s.textContent=a.phase+" | "+a.status+" | "+a.placed+"/5"},600)</script>','text/html; charset=utf-8')
        if path=='/':return self.reply((BASE/'competition.html').read_bytes(),'text/html; charset=utf-8')
        self.reply(b'not found',code=404)

    def do_POST(self):
        parsed=urlsplit(self.path)
        if parsed.path=='/api/exam-config':
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=4096:raise ValueError('配置大小不合法')
                config=validate_exam_config(json.loads(self.rfile.read(size)))
                with app.lock:
                    if app.busy:raise ValueError('运行期间不能修改考试配置')
                    app.exam_config=config;app.state['exam_config']=config
            except (ValueError,TypeError,KeyError) as error:
                return self.reply(json.dumps({'error':str(error)},ensure_ascii=False).encode(),code=400)
            return self.reply(b'{"configured":true}')
        if parsed.path=='/api/task':
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=65536:raise ValueError('题目 JSON 大小应为 1..65536 字节')
                app.set_official_task(json.loads(self.rfile.read(size)))
            except (ValueError,TypeError) as error:
                return self.reply(json.dumps({'error':str(error)},ensure_ascii=False).encode(),code=400)
            return self.reply(b'{"accepted":true}')
        if parsed.path=='/api/start':return self.reply(json.dumps({'started':app.start()}).encode())
        if parsed.path=='/api/stop':app.stop();return self.reply(b'{"stopped":true}')
        if parsed.path=='/api/observe':
            enabled=parse_qs(parsed.query).get('enabled',['1'])[0]=='1'
            return self.reply(json.dumps({'accepted':app.observation_session(enabled)}).encode())
        if parsed.path=='/api/reset':
            values=parse_qs(parsed.query)
            try:seed=int(values['seed'][0]) if 'seed' in values else None
            except (ValueError,IndexError):return self.reply(b'{"error":"invalid seed"}',code=400)
            mode=values.get('mode',['fixed'])[0]
            if mode not in ('distributed','fixed','easy'):
                return self.reply(b'{"error":"mode must be fixed, distributed or easy"}',code=400)
            if seed is None and mode=='fixed':seed=4
            if seed is None and mode=='easy':seed=20261019
            return self.reply(json.dumps({'resetting':app.reset_scene(seed,easy=mode=='easy',fixed=mode=='fixed')}).encode())
        self.reply(b'not found',code=404)


if __name__=='__main__':
    rclpy.init();app=Competition()
    def spin_ros():
        try:rclpy.spin(app)
        except rclpy.executors.ExternalShutdownException:pass
    spin_thread=threading.Thread(target=spin_ros,daemon=True)
    spin_thread.start()
    server=ThreadingHTTPServer(('127.0.0.1',8765),Handler)
    def shutdown(*_):
        app.stop()
        threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGTERM,shutdown)
    app.event('比赛观察面板已启动；模式：'+app.task_mode)
    try:server.serve_forever()
    finally:
        if rclpy.ok():
            app.stop();time.sleep(.2);rclpy.shutdown()
        spin_thread.join(timeout=2.)
        app.destroy_node()
