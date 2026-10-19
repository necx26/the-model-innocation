#!/usr/bin/env python3
"""Local practice controller. Real camera, Nav2, MoveIt and Gazebo joint attachment."""
import copy
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
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped, Point, Point32, PolygonStamped
from sensor_msgs.msg import Image, CameraInfo, JointState, LaserScan
from nav_msgs.msg import Path, OccupancyGrid
from rosgraph_msgs.msg import Clock
from nav2_msgs.action import NavigateToPose, Spin, BackUp
from nav2_msgs.msg import SpeedLimit
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
from mission_strategy import MissionMap, ObjectMemory, plan_deliveries
from model_adapter import invoke_official
from scene_config import SCENE, ZONES, PATROLS, in_delivery_area
from exam_bridge import generate_question, validate_exam_config

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
        self.frames = deque(maxlen=12)
        self.last_observed_stamp = None
        self.field = MissionMap(BASE/'src/yzbot/bot_navigation/maps/map.yaml')
        self.memory = ObjectMemory()
        self.search_seen = np.zeros(len(self.field.search_points), dtype=bool)
        self.search_attempted = {}
        self.observation_enabled = False
        self.perception_paused = False
        self.fixed_camera_images = {'overview': None, 'oblique': None}
        self.clock_time = 0.
        self.scan_min = 99.
        self.front_obstacle_range = 99.
        self.front_range_time = 0.
        self.front_range_history = deque(maxlen=3)
        self.last_front_sample = None
        self.front_closing_speed = 0.
        self.speed_limit_mps = float(SCENE.get('cruise_speed_mps', 1.0))
        self.speed_resume_since = None
        self.speed_policy_label = '巡航'
        self.slam_grid = None
        self.slam_grid_time = 0.
        self.nav_handle = None
        self.cancel = threading.Event()
        self.busy = False
        self.started = None
        self.used = set()
        self.held_model = None
        self.task_mode = os.environ.get('YZBOT_TASK_MODE', 'practice')
        if self.task_mode not in ('practice', 'official', 'exam'):
            raise ValueError('YZBOT_TASK_MODE 必须为 practice 或 official')
        self.official_input = None
        self.exam_config = validate_exam_config(dict(SCENE['exam']))
        self.trace, self.plan = [], []
        self.state = {'mode': '本地练习 · Gazebo 真值定位 · 官方接口待提供', 'phase': '就绪',
                      'status': '等待开始随机任务', 'question': '', 'task': None,
                      'picked': 0, 'placed': 0, 'total': 5, 'gripper': '已释放',
                      'events': [], 'detections': [], 'llm': '本地 Qwen 解析约束 + 算术工具精确求解',
                      'result': [], 'running': False, 'elapsed': 0., 'remaining': 300.}
        self.state['task_mode'] = self.task_mode
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
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', lambda m: setattr(self, 'info', m), qos_profile_sensor_data)
        for view in self.fixed_camera_images:
            self.create_subscription(Image, f'/fixed_{view}/image_raw',
                                     lambda m, name=view: self.fixed_image_cb(name, m),
                                     qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', lambda _: setattr(self, 'clock_time', time.monotonic()), qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', self.plan_cb, 10)
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
        self.create_subscription(EmptyMessage, '/competition/start_exam', lambda _: self.start(), 10)
        self.create_subscription(EmptyMessage, '/competition/stop', lambda _: self.stop(), 10)
        self.create_timer(.2, self.publish_overlay)
        self.command_pub = self.create_publisher(String, '/llama_command', qos)
        self.path_pub = self.create_publisher(Path, '/competition/path', 10)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.arm_action = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.spin_action = ActionClient(self, Spin, '/spin')
        self.backup_action = ActionClient(self, BackUp, '/backup')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.planner = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.attach = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach = self.create_client(DetachLink, '/DETACHLINK')
        self.reset = self.create_client(Empty, '/reset_world')
        self.set_entity = self.create_client(SetEntityState, '/demo/set_entity_state')
        self.create_timer(.5, self.publish_status)
        self.footprint_pub=self.create_publisher(PolygonStamped,'/competition/footprint',10)
        self.create_timer(.2,self.publish_footprint)
        self.create_timer(.2,self.observe_scene)
        self.create_timer(.2,self.update_dynamic_speed)

    def publish_footprint(self):
        # Body-relative, unstamped footprint remains valid after long arm work.
        msg=PolygonStamped();msg.header.frame_id='base_control'
        msg.polygon.points=[Point32(x=x,y=y) for x,y in [(-.33,-.23),(.15,-.23),(.15,.23),(-.33,.23)]]
        self.footprint_pub.publish(msg)

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

    def image_cb(self, msg):
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width*3]
        a = a.reshape(msg.height, msg.width, 3)
        self.image = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else a.copy()
        self.image_time = time.monotonic()
        self.image_stamp = msg.header.stamp
        self.frames.append((self.image,self.image_stamp,self.image_time))

    def fixed_image_cb(self, view, msg):
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width*3]
        a = a.reshape(msg.height, msg.width, 3)
        self.fixed_camera_images[view] = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else a.copy()

    def publish_overlay(self):
        if self.image is None or self.image_stamp is None:return
        img=self.image.copy()
        for d in self.state.get('detections',[]):
            x,y,w,h=d['box'];cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.putText(img,d['color'],(x,max(14,y-4)),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,0),1)
        msg=Image();msg.header.stamp=self.image_stamp;msg.header.frame_id='camera_link'
        msg.height,msg.width=img.shape[:2];msg.encoding='bgr8';msg.step=msg.width*3;msg.data=img.tobytes()
        self.annotated_pub.publish(msg)

    def observe_scene(self):
        if not self.observation_enabled or self.perception_paused or self.image_stamp is None:return
        try:
            detections,tf,image_stamp=self.vision(with_pose=True)
            stamp=(image_stamp.sec,image_stamp.nanosec)
            if stamp==self.last_observed_stamp:return
            self.last_observed_stamp=stamp
            if self.held_model:
                held_color=self.held_model.split('_')[0]
                detections=[d for d in detections if d['color']!=held_color]
            for target in self.memory.update(detections):
                self.event('相机发现并记住目标',target_id=target['id'],color=target['color'],
                           position=[target['x'],target['y']],source='onboard_camera')
            q=tf.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            indices=self.field.visible((tf.translation.x,tf.translation.y),yaw)
            if not self.held_model:self.search_seen[indices]=True
            self.state.pop('perception_note',None)
        except Exception as e:
            self.state['perception_note']=str(e)

    def scan_cb(self, msg):
        valid=[v for v in msg.ranges if math.isfinite(v) and msg.range_min < v < msg.range_max]
        self.scan_min=min(valid,default=99.)
        front=[]
        angle=msg.angle_min
        for value in msg.ranges:
            if abs((angle+math.pi)%(2*math.pi)-math.pi) <= .35 and math.isfinite(value) and msg.range_min < value < msg.range_max:
                front.append(float(value))
            angle+=msg.angle_increment
        if len(front)<3:
            return
        # A short percentile suppresses isolated lidar speckles while retaining a car
        # that occupies several neighboring rays in the forward corridor.
        distance=float(np.percentile(front,10))
        now=time.monotonic()
        self.front_range_history.append(distance)
        filtered=float(np.median(self.front_range_history))
        if self.last_front_sample:
            previous_time,previous_distance=self.last_front_sample
            dt=now-previous_time
            if .03<dt<1.0:
                raw_closing=(previous_distance-filtered)/dt
                # Patrol vehicles are slow relative to the 1 m/s base; cap scan-rate
                # spikes caused by turning past walls so they cannot create false TTCs.
                raw_closing=max(-1.3,min(1.3,raw_closing))
                self.front_closing_speed=.65*self.front_closing_speed+.35*raw_closing
        self.last_front_sample=(now,filtered)
        self.front_obstacle_range=filtered
        self.front_range_time=now

    def update_dynamic_speed(self):
        """TTC speed guard: use full cruise when clear/receding, yield when closing."""
        cruise=float(SCENE.get('cruise_speed_mps',1.0))
        active=bool(self.busy and self.nav_handle is not None)
        stale=time.monotonic()-self.front_range_time>.75
        distance=self.front_obstacle_range
        closing=max(0.,self.front_closing_speed)
        ttc=distance/closing if closing>.05 and distance<90 else float('inf')
        label='巡航/安全通行'
        proposed=cruise
        if active and stale:
            proposed=min(.25,cruise);label='雷达数据过期，保守限速'
        elif active:
            if distance<.48 or (ttc<.75 and distance<1.5):
                proposed=min(.10,cruise);label='紧急减速/等待'
            elif distance<.85 or (ttc<1.35 and distance<2.0):
                proposed=min(.30,cruise);label='减速让行'
            elif distance<1.35 or (ttc<2.0 and distance<2.6):
                proposed=min(.60,cruise);label='谨慎通过'
            elif self.front_closing_speed < -.18 and distance<1.8:
                proposed=cruise;label='障碍远离，恢复巡航'
        # Decelerate immediately; only restore speed after the forward gap stays safe.
        if proposed<self.speed_limit_mps-.03:
            self.speed_limit_mps=proposed
            self.speed_resume_since=None
        elif proposed>self.speed_limit_mps+.03:
            safe_gap=(distance>1.6 or self.front_closing_speed<-.18 or ttc>3.2)
            if not active or safe_gap:
                if self.speed_resume_since is None:self.speed_resume_since=time.monotonic()
                if time.monotonic()-self.speed_resume_since>=.6:
                    self.speed_limit_mps=proposed
                    self.speed_resume_since=None
            else:
                self.speed_resume_since=None
        else:
            self.speed_resume_since=None
        if self.speed_limit_mps<=.12:
            label='紧急减速/等待'
        elif self.speed_limit_mps<=.31:
            label='减速让行'
        elif self.speed_limit_mps<=.61:
            label='谨慎通过'
        old_label=self.state.get('dynamic_obstacle_policy')
        old_limit=self.state.get('dynamic_speed_limit_mps')
        self.state['dynamic_obstacle_policy']=label
        self.state['dynamic_speed_limit_mps']=round(self.speed_limit_mps,2)
        self.state['dynamic_front_range_m']=round(distance,2) if distance<90 else None
        self.state['dynamic_closing_speed_mps']=round(self.front_closing_speed,2)
        self.state['dynamic_ttc_s']=round(ttc,2) if math.isfinite(ttc) else None
        if label!=old_label or old_limit is None or abs(self.speed_limit_mps-float(old_limit))>=.1:
            self.event('动态障碍速度决策',decision=label,speed_limit_mps=round(self.speed_limit_mps,2),
                       front_range_m=round(distance,2),closing_speed_mps=round(self.front_closing_speed,2),
                       ttc_s=round(ttc,2) if math.isfinite(ttc) else None)
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
        self.get_logger().info(text)

    def update(self, phase, text):
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
                 search_bounds=self.field.bounds, zones=ZONES, patrols=PATROLS, exam_config=self.exam_config,
                 camera_age=round(time.monotonic()-self.image_time, 1),
                 robot=self.robot_pose(),
                 cubes=[{'name': n, 'x': p.position.x, 'y': p.position.y, 'z': p.position.z}
                        for n, p in self.models.items() if n.startswith(('red_cube', 'blue_cube'))],
                 obstacles=[{'name': n, 'x': p.position.x, 'y': p.position.y}
                            for n, p in self.models.items() if n in ['Move1', 'Move2']])
        issues=[]
        if s['robot'] is None:issues.append('缺少 odom→base_control 坐标变换')
        if self.image is None or s['camera_age']>3:issues.append('前置相机没有新图像')
        if time.monotonic()-self.clock_time>3:issues.append('Gazebo 仿真时钟停止')
        if self.state['localization']=='slam' and (self.slam_grid is None or time.monotonic()-self.slam_grid_time>5):issues.append('SLAM 地图没有更新')
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

    def check(self):
        if self.cancel.is_set():
            raise RuntimeError('任务已停止')
        if self.started and self.elapsed() >= 300:
            self.vel.publish(Twist())
            if self.nav_handle:
                self.nav_handle.cancel_goal_async()
            raise RuntimeError('已达到 5 分钟时限，停止作业')

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

    def go(self, x, y, yaw, look_for=None):
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
        self.nav_handle = self.wait(self.nav.send_goal_async(goal), 10)
        if not self.nav_handle.accepted:
            raise RuntimeError('Nav2 拒绝目标')
        handle=self.nav_handle
        future=handle.get_result_async()
        end=time.monotonic()+65
        try:
            while not future.done():
                self.check()
                if look_for and any(self.field.approaches(t) for t in self.memory.available(look_for)):
                    self.wait(handle.cancel_goal_async(),5)
                    self.wait(future,8)
                    self.vel.publish(Twist())
                    return False
                if time.monotonic()>end:raise TimeoutError('导航到达超时')
                time.sleep(.1)
            result=future.result()
        finally:
            if not future.done():handle.cancel_goal_async()
            self.nav_handle=None
        if result.status != 4:
            raise RuntimeError(f'Nav2 未到达目标，状态 {result.status}')
        self.align(x, y, yaw)
        return True

    def scan_compartment(self, color, intended):
        # Sweep the shorter arc covering only still-unseen map cells.
        time.sleep(.25)
        unseen=intended[~self.search_seen[intended]]
        if not len(unseen):return
        tf=self.buffer.lookup_transform('odom','camera_link',Time()).transform
        q=tf.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        delta=self.field.search_points[unseen]-np.array([tf.translation.x,tf.translation.y])
        bearings=(np.arctan2(delta[:,1],delta[:,0])-yaw+math.pi)%(2*math.pi)-math.pi
        positive=np.where(np.abs(bearings)<.66,0.,(bearings-.66)%(2*math.pi))
        negative=np.where(np.abs(bearings)<.66,0.,(-bearings-.66)%(2*math.pi))
        direction=1. if np.max(positive)<=np.max(negative) else -1.
        remaining=float(min(np.max(positive),np.max(negative)))+.12
        while remaining>.05:
            if any(self.field.approaches(t) for t in self.memory.available(color)):return
            if not np.any(~self.search_seen[intended]):return
            if not self.spin_action.wait_for_server(timeout_sec=3):
                raise RuntimeError('搜索旋转服务不可用')
            turn=min(math.pi/2,remaining);remaining-=turn
            command=Spin.Goal(target_yaw=float(direction*turn),time_allowance=Duration(sec=12))
            handle=self.wait(self.spin_action.send_goal_async(command),5)
            if not handle.accepted:raise RuntimeError('Nav2 拒绝搜索旋转')
            self.nav_handle=handle;future=handle.get_result_async()
            while not future.done():
                self.check()
                if any(self.field.approaches(t) for t in self.memory.available(color)) or not np.any(~self.search_seen[intended]):
                    self.wait(handle.cancel_goal_async(),5);self.wait(future,8)
                    self.nav_handle=None;return
                time.sleep(.1)
            self.nav_handle=None
            if future.result().status!=4:
                self.event('搜索机位旋转受阻，改选其他机位');return
            time.sleep(.2)

    def align(self, x, y, yaw):
        end = time.monotonic()+25
        while time.monotonic() < end:
            self.check()
            p = self.robot_pose()
            if p is None:
                time.sleep(.1)
                continue
            d = math.hypot(x-p[0], y-p[1])
            target = math.atan2(y-p[1], x-p[0]) if d > .045 else yaw
            e = angle(target-p[2])
            direction=1.
            if d>.045 and abs(e)>math.pi/2:
                direction=-1.;e=angle(e+math.pi)
            if d < .045 and abs(angle(yaw-p[2])) < .07:
                self.vel.publish(Twist())
                return
            cmd = Twist()
            cmd.angular.z = float(max(-.8, min(.8, 2.5*e)))
            if d > .045 and abs(e) < .25:
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

    def solve_ik(self, x, y, z):
        t = self.buffer.lookup_transform('base_footprint', 'odom', Time()).transform
        p = rotation(t.rotation)@np.array([x,y,z])+np.array([t.translation.x,t.translation.y,t.translation.z])
        radial=math.hypot(p[0],p[1]); height=p[2]+.06-.2834
        cosine=(radial*radial+height*height-.14**2-.28**2)/(2*.14*.28)
        analytic=None
        if -1<cosine<1:
            elbow=math.acos(cosine)
            shoulder=math.atan2(radial,height)-math.atan2(.28*math.sin(elbow),.14+.28*math.cos(elbow))
            phi=math.atan2(p[1],p[0])
            analytic=[phi,shoulder,elbow,0.,shoulder+elbow-math.pi,phi]
        for seed in range(8):
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
            pose.orientation.y=1.; pose.orientation.w=0.
            req.ik_request.timeout=Duration(sec=1)
            r=self.call(self.ik,req,5)
            if r.error_code.val==1:
                data=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position))
                return [data[j] for j in JOINTS]
        raise RuntimeError(f'MoveIt IK 无可行解: {p.round(3).tolist()}')

    def arm(self, values):
        req=GetMotionPlan.Request()
        r=req.motion_plan_request
        r.group_name='arm'; r.start_state=self.robot_state()
        r.allowed_planning_time=3.; r.num_planning_attempts=5
        r.max_velocity_scaling_factor=.9; r.max_acceleration_scaling_factor=.9
        c=Constraints()
        for j,v in zip(JOINTS,values):
            c.joint_constraints.append(JointConstraint(joint_name=j,position=float(v),tolerance_above=.01,tolerance_below=.01,weight=1.))
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
        pending=handle.get_result_async()
        duration=traj.points[-1].time_from_start
        deadline=time.monotonic()+max(8,duration.sec+duration.nanosec/1e9+5)
        stable=0
        while time.monotonic()<deadline:
            self.check()
            err=max(abs(self.joints.get(j,99)-v) for j,v in zip(JOINTS,values))
            if pending.done():
                outcome=pending.result()
                if outcome.status==4 and err<.06:return
                if outcome.status!=4:raise RuntimeError('机械臂动作失败：'+outcome.result.error_string)
            stable=stable+1 if err<.04 else 0
            if stable>=5:return
            time.sleep(.08)
        raise RuntimeError(f'机械臂跟踪未达到目标，最大关节误差 {err:.3f}')

    def arm_xyz(self,x,y,z):
        self.arm(self.solve_ik(x,y,z))

    def retreat_after_drop(self):
        """Back away from a placed object after the arm has been stowed."""
        if not self.backup_action.wait_for_server(timeout_sec=2):
            self.event('放置后未能后退',reason='Nav2 BackUp 行为不可用')
            return False
        for distance in (.30, .15):
            if self.cancel.is_set() or (self.started and self.elapsed() >= 300):
                return False
            try:
                goal=BackUp.Goal(target=Point(x=distance),speed=.20,
                                 time_allowance=Duration(sec=8))
                handle=self.wait(self.backup_action.send_goal_async(goal),5)
                self.nav_handle=handle
                if not handle.accepted:
                    self.event('放置后后退被拒绝',distance=distance)
                    self.nav_handle=None
                    continue
                result=self.wait(handle.get_result_async(),10)
                self.nav_handle=None
                if result.status==4:
                    self.event('放置后安全后退',distance=distance,speed=.20)
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

    def grip(self, closed):
        m=JointTrajectory(joint_names=['finger_joint1'])
        m.points=[JointTrajectoryPoint(positions=[.015 if closed else 0.],time_from_start=Duration(sec=1))]
        self.grip_pub.publish(m)
        time.sleep(.6)
        self.state['gripper']='闭合' if closed else '张开'

    def vision(self, with_pose=False):
        if self.image is None or self.info is None or time.monotonic()-self.image_time>3:
            raise RuntimeError('相机没有有效图像')
        frame=None
        for candidate in reversed(list(self.frames)):
            if time.monotonic()-candidate[2]<2 and self.buffer.can_transform('odom','camera_link',Time.from_msg(candidate[1])):
                frame=candidate;break
        if frame is None:raise RuntimeError('等待与相机帧同一时间的 TF')
        img=frame[0].copy();image_stamp=frame[1]
        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        masks={'red':cv2.inRange(hsv,(0,100,65),(12,255,255))|cv2.inRange(hsv,(168,100,65),(179,255,255)),
               'blue':cv2.inRange(hsv,(100,100,45),(135,255,255))}
        tf=self.buffer.lookup_transform('odom','camera_link',Time.from_msg(image_stamp)).transform
        rot=rotation(tf.rotation); org=np.array([tf.translation.x,tf.translation.y,tf.translation.z])
        k=self.info.k; result=[]
        for color,mask in masks.items():
            contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt)<8:continue
                x,y,w,h=cv2.boundingRect(cnt)
                if not .3<w/max(1,h)<3:continue
                # Ground-contact pixel remains valid when the top of a nearby cube is clipped.
                if y+h>=img.shape[0]-1:continue
                u,v=x+w/2,y+h-1
                ray=rot@np.array([1.,-(u-k[2])/k[0],-(v-k[5])/k[4]])
                if ray[2]>=-.015:continue
                point=org+ray*((0.-org[2])/ray[2])
                # The mask's bottom edge is the near face of a 5 cm cube.
                # Move half a side away from the camera to estimate its centre.
                axis=int(np.argmax(np.abs(point[:2]-org[:2])))
                point[axis]+=.025*np.sign(point[axis]-org[axis])
                if np.linalg.norm(point[:2]-org[:2])>2.5:continue
                rp=self.robot_pose()
                if rp and math.hypot(point[0]-rp[0],point[1]-rp[1])<.40:continue
                if in_delivery_area(point, extra=.1):continue
                cell=self.field.cell(point)
                if not self.field.inside(cell) or self.field.clearance[cell]<.08:continue
                result.append({'color':color,'x':float(point[0]),'y':float(point[1]),'z':.025,'pixel':[u,v],
                               'box':[x,y,w,h],'source':'camera_HSV_ground_projection'})
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
        if self.busy:return False
        if self.task_mode == 'official' and self.official_input is None:
            self.state['status']='尚未收到官方题目，请先 POST /api/task';return False
        if self.held_model or self.used:
            self.state['status']='请先重置场景再开始新一轮';return False
        if self.robot_pose() is None or self.image is None or not self.models:
            self.state['status']='等待相机、里程计与仿真状态连接';return False
        self.cancel.clear(); self.busy=True; self.started=time.monotonic()
        self.used=set(); self.trace=[]
        self.memory=ObjectMemory();self.search_seen[:]=False;self.search_attempted={}
        self.perception_paused=False
        self.last_observed_stamp=None;self.observation_enabled=True
        self.state.update(running=True,picked=0,placed=0,result=[],events=[],task=None)
        threading.Thread(target=self.run,daemon=True).start()
        return True

    def stop(self):
        self.cancel.set(); self.vel.publish(Twist()); self.stop_vel.publish(Twist())
        if self.nav_handle:self.nav_handle.cancel_goal_async()

    def reset_scene(self, seed=None):
        if self.busy:return False
        self.busy=True;self.cancel.clear();self.started=None
        self.observation_enabled=False
        self.state.update(phase='准备场景',status='正在复位并随机摆放方块')
        def reset_worker():
            try:
                self.stop_vel.publish(Twist());self.vel.publish(Twist())
                if self.held_model:
                    self.call(self.detach,DetachLink.Request(model1_name='six_arm',link1_name='link6',model2_name=self.held_model,link2_name='link'))
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
                layout_seed=random.SystemRandom().getrandbits(32) if seed is None else int(seed)
                layout=self.field.scattered_layout(layout_seed)
                for item in layout['layout']:
                    entity=EntityState(name=item['model'],reference_frame='world')
                    entity.pose.position.x=item['x'];entity.pose.position.y=item['y'];entity.pose.position.z=.025
                    entity.pose.orientation.w=1.
                    if not self.call(self.set_entity,SetEntityState.Request(state=entity)).success:
                        raise RuntimeError('随机摆放失败: '+entity.name)
                self.state['layout_seed']=layout_seed
                self.state['layout_randomized']=True
                self.state['layout_mode']='distributed_office'
                self.state['layout_sector_count']=layout['sector_count']
                (LOG/'random-layout.json').write_text(json.dumps(layout,ensure_ascii=False,indent=2))
                time.sleep(.5)
                actual=self.models.get('six_arm')
                if actual is None or math.hypot(actual.position.x,actual.position.y)>.1:
                    raise RuntimeError('复位位置核验未通过')
                # Practice uses Gazebo world odometry and a static map alignment.
                self.state.update(phase='就绪',status='方块已随机摆放，场景已重置，可开始新一轮',question='',task=None,picked=0,placed=0,result=[],elapsed=0,remaining=300,running=False)
                self.used=set();self.trace=[];self.memory=ObjectMemory();self.search_seen[:]=False;self.search_attempted={}
                self.event('场景已复位，红蓝方块分散到七个地图分区',seed=layout_seed)
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
            route=plan_deliveries(self.field,rp,candidates,remaining,zones,number)
            if route:
                self.state['known_plan']=[{'id':r['target']['id'],'color':r['target']['color'],
                                          'x':r['target']['x'],'y':r['target']['y'],
                                          'seconds':r['estimated_seconds']} for r in route]
                selected=route[0]
                self.event('按可通行路径选择搬运目标',target_id=selected['target']['id'],
                           known_plan=self.state['known_plan'],estimated_seconds=selected['known_plan_seconds'])
                return selected
            view=self.field.next_view(rp,self.search_seen,self.search_attempted,delivery=zones[color])
            if view is None:
                if exhausted:raise RuntimeError('已搜索可达区域，未发现足够的当前颜色目标')
                self.search_seen[:]=False;self.search_attempted={};exhausted=True
                self.event('首次搜索未满足数量，重新检查遗漏区域');continue
            _,idx,pose,gain,cost=view
            self.search_attempted[idx]=self.search_attempted.get(idx,0)+1
            self.state['search_goal']={'x':pose[0],'y':pose[1],'yaw':pose[2],'new_cells':gain}
            label='红色' if color=='red' else '蓝色'
            self.update('搜索隔间',f'寻找{label}目标，前往观察点 ({pose[0]:.1f}, {pose[1]:.1f})')
            try:
                reached=self.go(*pose,look_for=color)
                if reached:
                    self.update('环视识别',f'在隔间观察点检查{label}方块')
                    self.scan_compartment(color,self.field.views[idx]['visible'])
            except Exception as e:
                self.check()
                self.event('观察点暂时不可达，重新选择',reason=str(e))

    def approach_target(self, selection):
        target=selection['target']
        self.update('前往目标',f"前往相机记录的 {target['color']} #{target['id']}")
        self.go(*selection['view'])
        time.sleep(.35)
        detections=[d for d in self.vision() if d['color']==target['color']
                    and math.hypot(d['x']-target['x'],d['y']-target['y'])<.30]
        if not detections:raise RuntimeError('抵近后相机未重新确认目标')
        d=min(detections,key=lambda d:math.hypot(d['x']-target['x'],d['y']-target['y']))
        yaw=selection['grasp'][2]
        grasp=(d['x']-.20*math.cos(yaw),d['y']-.20*math.sin(yaw),yaw)
        if not self.field.safe(grasp):raise RuntimeError('精对准抓取位姿靠墙，换其他目标')
        self.align(*grasp)
        # Simulator identity is resolved only here, after image-guided arrival.
        # It supplies attachment identity/contact verification, never a search goal.
        available=[(name,p) for name,p in self.models.items()
                   if name.startswith(target['color']+'_cube') and name not in self.used]
        if not available:raise RuntimeError('没有可用于仿真夹持核验的实体')
        name,p=min(available,key=lambda item:math.hypot(item[1].position.x-d['x'],item[1].position.y-d['y']))
        error=math.hypot(p.position.x-d['x'],p.position.y-d['y'])
        if error>.12:raise RuntimeError(f'相机定位与近处实体不一致 {error:.3f}m')
        self.event('相机抵近确认目标',target_id=target['id'],model=name,detection=d,grasp_pose=grasp)
        return name,d

    def run(self):
        try:
            self.update('大模型解题','正在运行出题程序并调用模型解题' if self.task_mode=='exam' else ('等待官方模型适配器解题' if self.task_mode=='official' else '随机题目已生成，等待本地 Qwen 解题'))
            task=self.llm_task()
            self.check()
            self.event('大模型已输出并通过校验',answer=task)
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
                    try:
                        name,d=self.approach_target(selection)
                        break
                    except Exception as e:
                        self.check()
                        self.memory.fail(selection['target']['id'])
                        self.event('目标抵近失败，更新候选代价',target_id=selection['target']['id'],reason=str(e))
                self.perception_paused=True
                self.grip(False)
                self.update('抓取',f'机械臂正在抓取 {name}')
                self.arm_xyz(d['x'],d['y'],.16)
                self.arm_xyz(d['x'],d['y'],.085)
                p=self.models[name].position;hand=self.links.get('six_arm::link6')
                if hand is None:raise RuntimeError('缺少真实末端 link6 位姿')
                distance=math.dist([p.x,p.y,p.z],[hand.position.x,hand.position.y,hand.position.z])
                if distance>.105:raise RuntimeError(f'末端距货物 {distance:.3f}m，禁止远距离附着')
                self.grip(True)
                req=AttachLink.Request(model1_name='six_arm',link1_name='link6',model2_name=name,link2_name='link')
                r=self.call(self.attach,req)
                if not r.success:raise RuntimeError(r.message)
                self.held_model=name;self.state['gripper']='已抓住 '+name
                self.arm_xyz(d['x'],d['y'],.32);time.sleep(.3)
                if self.models[name].position.z<.12:raise RuntimeError('货物未随机械臂抬升')
                self.state['picked']+=1;self.used.add(name)
                self.memory.mark(selection['target']['id'],'picked')
                self.event('抓取并抬升确认',model=name,contact_distance=distance,z=self.models[name].position.z)
                rp=self.robot_pose()
                self.arm_xyz(rp[0]+.10*math.cos(rp[2]),rp[1]+.10*math.sin(rp[2]),.43)
                self.perception_paused=False
                zone=task[color+'_zone'];zx,zy=ZONES[zone]
                px=zx+.10*(number%3);py=zy+.08*(number//3-.5)
                self.update('搬运',f'携带 {name} 前往 {zone} 区')
                self.go(px-.20,py,0.)
                self.update('放置',f'正在把 {name} 放入 {zone} 区')
                self.perception_paused=True
                self.arm_xyz(px,py,.23);self.arm_xyz(px,py,.12)
                req=DetachLink.Request(model1_name='six_arm',link1_name='link6',model2_name=name,link2_name='link')
                r=self.call(self.detach,req)
                if not r.success:raise RuntimeError(r.message)
                self.held_model=None;self.grip(False);time.sleep(.6)
                p=self.models[name].position
                valid=abs(p.x-zx)<.5 and abs(p.y-zy)<.25 and p.z<.16
                if not valid:raise RuntimeError(f'货物未落在指定 {zone} 区')
                self.state['placed']+=1;self.memory.mark(selection['target']['id'],'delivered')
                result={'model':name,'zone':zone,'position':[p.x,p.y,p.z],'verified':True}
                self.state['result'].append(result);self.event('放置位置确认',**result)
                self.arm_xyz(px,py,.30)
                rp=self.robot_pose()
                self.arm_xyz(rp[0]+.10*math.cos(rp[2]),rp[1]+.10*math.sin(rp[2]),.43)
                # The placed item is in front of the chassis. Retreat before starting
                # the next route so the base and arm do not sweep through the drop zone.
                self.retreat_after_drop()
                self.perception_paused=False
            self.update('完成','5 个分散货物已按指令完成搜索、抓取和放置')
        except Exception as e:
            self.update('停止' if self.cancel.is_set() else '未完成',str(e))
        finally:
            self.vel.publish(Twist());self.observation_enabled=False
            if self.nav_handle:self.nav_handle.cancel_goal_async()
            self.state.update(running=False,elapsed=round(self.elapsed(),1),remaining=max(0,round(300-self.elapsed(),1)))
            self.busy=False
            (LOG/'competition-result.json').write_text(json.dumps(self.snapshot(),ensure_ascii=False,indent=2))


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
            image=app.image.copy()
            for d in app.state['detections'] if path=='/api/image' else []:
                x,y,w,h=d['box'];cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
            return self.reply(cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,75])[1].tobytes(),'image/jpeg')
        if path in ['/api/camera/overview', '/api/camera/oblique']:
            view=path.rsplit('/',1)[-1]
            image=app.fixed_camera_images.get(view)
            if image is None:return self.reply(b'',kind='image/jpeg',code=503)
            return self.reply(cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,75])[1].tobytes(),'image/jpeg')
        if path=='/api/slam-map':
            try:
                return self.reply(json.dumps(app.map_view(),ensure_ascii=False).encode())
            except Exception as error:
                return self.reply(json.dumps({'error':str(error)},ensure_ascii=False).encode(),code=503)
        if path=='/api/map':
            raw=cv2.imread(str(BASE/'src/yzbot/bot_navigation/maps/map.pgm'),0)
            # Real occupancy map crop: x -4..5, y -3..3, 0.05m/pixel, origin -50,-50.
            xmin,xmax,ymin,ymax=app.field.bounds
            raw=raw[raw.shape[0]-1-round((ymax+50)/.05):raw.shape[0]-round((ymin+50)/.05),round((xmin+50)/.05):round((xmax+50)/.05)+1]
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
        if parsed.path=='/api/reset':
            values=parse_qs(parsed.query)
            try:seed=int(values['seed'][0]) if 'seed' in values else None
            except (ValueError,IndexError):return self.reply(b'{"error":"invalid seed"}',code=400)
            return self.reply(json.dumps({'resetting':app.reset_scene(seed)}).encode())
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
