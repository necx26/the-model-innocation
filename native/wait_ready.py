import argparse
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, CompressedImage
from nav_msgs.msg import OccupancyGrid
from lifecycle_msgs.srv import GetState
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetMotionPlan
from tf2_ros import Buffer, TransformListener

p=argparse.ArgumentParser();p.add_argument('stage',choices=['gazebo','nav']);p.add_argument('--timeout',type=float,default=150);a=p.parse_args()
rclpy.init();n=Node('yzbot_wait_'+a.stage,parameter_overrides=[Parameter('use_sim_time',value=True)])
seen=set();last={};end=time.monotonic()+a.timeout;missing=[]
def received(name):
    seen.add(name);last[name]=time.monotonic()
n.create_subscription(Clock,'/clock',lambda _:received('clock'),qos_profile_sensor_data)
n.create_subscription(JointState,'/joint_states',lambda _:received('joints'),qos_profile_sensor_data)
arm=ActionClient(n,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory')
if a.stage=='nav':
    tf=Buffer(node=n);listener=TransformListener(tf,n)
    camera_stamps=[]
    def camera_received(msg):
        received('camera')
        stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        if not camera_stamps or stamp!=camera_stamps[-1]:camera_stamps.append(stamp)
        del camera_stamps[:-20]
    camera_sub=n.create_subscription(CompressedImage,'/camera/image_raw/compressed',camera_received,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
    n.create_subscription(OccupancyGrid,'/map',lambda _:received('map'),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    clients={s:n.create_client(GetState,'/'+s+'/get_state') for s in ['bt_navigator','controller_server']};futures={}
    planner=n.create_client(GetMotionPlan,'/plan_kinematic_path')

def ready():
    missing[:]=[name for name in ['clock','joints'] if time.monotonic()-last.get(name,0)>3]
    if not arm.server_is_ready():missing.append('arm_action')
    if a.stage=='nav':
        for name,c in clients.items():
            if name not in seen and name not in futures and c.service_is_ready():futures[name]=c.call_async(GetState.Request())
        for name,f in list(futures.items()):
            if f.done():
                if f.result() and f.result().current_state.id==3:seen.add(name)
                del futures[name]
        missing.extend(name for name in clients if name not in seen)
        for name in ['camera','map']:
            if time.monotonic()-last.get(name,0)>5:missing.append(name)
        if len(camera_stamps)<5:missing.append('continuous_camera_frames')
        elif not any(tf.can_transform('odom','camera_link',Time(seconds=sec,nanoseconds=nsec)) for sec,nsec in camera_stamps[-10:]):missing.append('camera_timestamp_tf')
        if not planner.service_is_ready():missing.append('moveit_planner')
        for frame in ['odom','map']:
            try:
                t=tf.lookup_transform(frame,'base_control',Time())
                age=(n.get_clock().now().nanoseconds-(t.header.stamp.sec*10**9+t.header.stamp.nanosec))/1e9
                if not -.5<age<3:missing.append(frame+'_tf_stale')
            except Exception:missing.append(frame+'_tf')
    return not missing
try:
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.1)
        if ready():print(a.stage+' ready (clock, robot control'+', TF, camera, SLAM, MoveIt, Nav2'*(a.stage=='nav')+')',flush=True);break
    else:raise SystemExit(a.stage+' readiness timeout; missing='+str(missing))
finally:n.destroy_node();rclpy.shutdown()
