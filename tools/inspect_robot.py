#!/usr/bin/env python3
"""Capture a temporary Gazebo close-up; remove the diagnostic camera afterward."""
import math
import sys
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from sensor_msgs.msg import Image

rclpy.init()
node=Node('yzbot_closeup_check',parameter_overrides=[Parameter('use_sim_time',value=True)])
models={};frames=[]
node.create_subscription(ModelStates,'/demo/model_states_demo',lambda msg:models.update(zip(msg.name,msg.pose)),10)
subscriptions=[node.create_subscription(Image,topic,lambda msg:frames.append(msg),qos_profile_sensor_data) for topic in ['/yzbot_inspect/image_raw','/yzbot_inspect/inspection/image_raw']]
def until(test,seconds=15):
    end=time.monotonic()+seconds
    while not test() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    if not test():raise RuntimeError('Gazebo close-up timed out')
def call(kind,name,request):
    c=node.create_client(kind,name)
    if not c.wait_for_service(timeout_sec=5):raise RuntimeError(name+' unavailable')
    f=c.call_async(request);until(f.done);return f.result()
try:
    until(lambda:'six_arm' in models)
    p=models['six_arm'].position
    q=models['six_arm'].orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    cx=p.x+.65*math.cos(yaw);cy=p.y+.65*math.sin(yaw)
    camera_yaw=yaw+math.pi
    xml=f'''<sdf version="1.6"><model name="yzbot_diagnostic_camera"><static>true</static>
      <pose>{cx} {cy} .8 0 .65 {camera_yaw}</pose><link name="camera">
      <sensor name="inspection" type="camera"><always_on>true</always_on><update_rate>5</update_rate>
      <camera><horizontal_fov>1.1</horizontal_fov><image><width>960</width><height>640</height><format>R8G8B8</format></image>
      <clip><near>.02</near><far>20</far></clip></camera>
      <plugin name="inspection_ros" filename="libgazebo_ros_camera.so"><ros><namespace>/yzbot_inspect</namespace><remapping>~/image_raw:=/yzbot_inspect/image_raw</remapping></ros>
      <camera_name>inspection</camera_name><frame_name>inspection</frame_name></plugin></sensor></link></model></sdf>'''
    r=call(SpawnEntity,'/spawn_entity',SpawnEntity.Request(name='yzbot_diagnostic_camera',xml=xml))
    if not r.success:raise RuntimeError(r.status_message)
    until(lambda:len(frames)>=3)
    msg=frames[-1];a=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
    if msg.encoding=='rgb8':a=cv2.cvtColor(a,cv2.COLOR_RGB2BGR)
    out=Path(sys.argv[1]);cv2.imwrite(str(out),a);print(out)
finally:
    call(DeleteEntity,'/delete_entity',DeleteEntity.Request(name='yzbot_diagnostic_camera'))
    node.destroy_node();rclpy.shutdown()
