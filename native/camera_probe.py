#!/usr/bin/env python3
"""Measure the cameras in the running OS; no downloads or system changes."""
import argparse
import collections
import json
import os
from pathlib import Path
import platform
import time
import urllib.request
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image
from tf2_ros import Buffer, TransformListener

parser=argparse.ArgumentParser()
parser.add_argument('--seconds',type=float,default=15)
parser.add_argument('--compare-raw',action='store_true',help='Also subscribe to old raw transport for a comparison')
args=parser.parse_args()
cv2.setNumThreads(1)
rclpy.init()
node=Node('competition_camera_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
buffer=Buffer(node=node);listener=TransformListener(buffer,node)
stats={};pending=collections.deque(maxlen=30)
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
def receive(name,msg,compressed=True):
    now=time.monotonic();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    s=stats.setdefault(name,dict(frames=0,unique=0,bytes=0,first_stamp=stamp,last_stamp=stamp,gaps=[],decode_errors=0,last_progress=now))
    s['frames']+=1;s['bytes']=len(msg.data)
    if s['unique']==0 or stamp!=s['last_stamp']:
        if s['unique']:s['gaps'].append(round(stamp-s['last_stamp'],4))
        s['unique']+=1;s['last_progress']=now
    s['last_stamp']=stamp
    if compressed:
        image=cv2.imdecode(np.frombuffer(msg.data,dtype=np.uint8),cv2.IMREAD_COLOR)
        if image is None:s['decode_errors']+=1
        elif name=='front':
            hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
            red=cv2.inRange(hsv,(0,100,65),(12,255,255))|cv2.inRange(hsv,(168,100,65),(179,255,255))
            blue=cv2.inRange(hsv,(100,100,45),(135,255,255))
            s['color_pixels']=dict(red=int(np.count_nonzero(red)),blue=int(np.count_nonzero(blue)))
            pending.append(msg.header.stamp)
subscriptions=[]
for name,topic in [('front','/camera'),('overview','/fixed_overview'),('oblique','/fixed_oblique')]:
    subscriptions.append(node.create_subscription(CompressedImage,topic+'/image_raw/compressed',lambda m,n=name:receive(n,m),qos))
    if args.compare_raw:subscriptions.append(node.create_subscription(Image,topic+'/image_raw',lambda m,n=name:receive(n+'_raw',m,False),qos_profile_sensor_data))
start=time.monotonic();tf_ok=0;tf_fail=0
while time.monotonic()-start<args.seconds:
    rclpy.spin_once(node,timeout_sec=.05)
    # Wait for TF to catch up, instead of checking only in the image callback.
    while len(pending)>5:
        stamp=pending.popleft()
        if buffer.can_transform('odom','camera_link',Time.from_msg(stamp)):tf_ok+=1
        else:tf_fail+=1
now=time.monotonic()
for name,s in stats.items():
    gaps=s.pop('gaps');s['max_sim_gap']=max(gaps,default=0);s['age']=round(now-s.pop('last_progress'),3)
    s['simulation_span']=round(s['last_stamp']-s['first_stamp'],3)
    s['wall_fps']=round(s['unique']/(now-start),2)
    s['expected_period']=.2 if name.startswith(('overview','oblique')) else .1
    s['estimated_missed_frames']=sum(max(0,round(g/s['expected_period'])-1) for g in gaps)
report=dict(date=time.strftime('%Y-%m-%d %H:%M:%S'),kernel=platform.release(),project=os.environ.get('YZBOT_PROJECT'),
            dds=os.environ.get('RMW_IMPLEMENTATION'),dds_config=os.environ.get('CYCLONEDDS_URI'),duration=round(now-start,2),
            tf_samples_ok=tf_ok,tf_samples_failed=tf_fail,cameras=stats)
for key in ['rmem_max','wmem_max']:
    try:report[key]=int(Path('/proc/sys/net/core/'+key).read_text())
    except OSError:pass
try:report['controller']=json.load(urllib.request.urlopen('http://127.0.0.1:8765/api/state',timeout=3))
except Exception as error:report['controller_error']=str(error)
report['passed']=all(stats.get(n,{}).get('unique',0)>=10 and stats[n]['age']<2 and stats[n]['decode_errors']==0 for n in ['front','overview','oblique']) and tf_ok>5
log=Path(os.environ.get('YZBOT_LOG',str(Path(__file__).resolve().parents[1]/'logs')))
log.mkdir(exist_ok=True,parents=True);target=log/'camera-diagnostic-latest.json'
target.write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps({k:v for k,v in report.items() if k!='controller'},ensure_ascii=False,indent=2))
print('诊断已保存：'+str(target));node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if report['passed'] else 1)
