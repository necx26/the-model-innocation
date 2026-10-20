#!/usr/bin/env python3
"""Compare two bounded, collision-checked Nav2 heading corrections at rest."""
import json,math,time,urllib.request
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
rclpy.init();n=Node('heading_comparison');samples=[]
n.create_subscription(Odometry,'/odom',lambda m:samples.append((time.monotonic(),m.pose.pose)),10)
c=n.create_client(SetParameters,'/controller_server/set_parameters');a=ActionClient(n,NavigateToPose,'/navigate_to_pose')
def wait(f,timeout=15):
 rclpy.spin_until_future_complete(n,f,timeout_sec=timeout)
 if not f.done():raise TimeoutError('Nav2 comparison timeout')
 return f.result()
def speed(value):
 req=SetParameters.Request(parameters=[Parameter(name='FollowPath.rotate_to_heading_angular_vel',value=ParameterValue(type=3,double_value=value))])
 result=wait(c.call_async(req));assert result.results[0].successful,result.results[0].reason
assert c.wait_for_service(timeout_sec=5) and a.wait_for_server(timeout_sec=5)
s=json.load(urllib.request.urlopen('http://127.0.0.1:8765/api/state'));assert not s['busy'] and s['placed']==5
x,y,yaw=s['robot'];records=[]
def turn(target,value,record=True):
 speed(value);g=NavigateToPose.Goal();g.pose.header.frame_id='odom';g.pose.pose.position.x=x;g.pose.pose.position.y=y
 g.pose.pose.orientation.z=math.sin(target/2);g.pose.pose.orientation.w=math.cos(target/2)
 start=time.monotonic();before=len(samples);h=wait(a.send_goal_async(g));assert h.accepted
 pending=h.get_result_async()
 try:result=wait(pending,15)
 finally:
  if not pending.done():wait(h.cancel_goal_async(),3)
 assert result.status==4,result.status
 position=json.load(urllib.request.urlopen('http://127.0.0.1:8765/api/state'))['robot'];distance=math.hypot(position[0]-x,position[1]-y)
 row={'angular_speed_setting':value,'seconds':time.monotonic()-start,'navigation_status':result.status,'translation_m':distance,'odometry_samples':len(samples)-before}
 assert distance<.05,row
 if record:records.append(row)
try:
 turn(yaw+math.pi/2,1.0);turn(yaw,1.0,False);turn(yaw+math.pi/2,1.4)
finally:speed(1.0);n.destroy_node();rclpy.shutdown()
out=Path('/opt/yzbot/verification/heading-comparison.json');out.write_text(json.dumps({'purpose':'same base_control location and heading target; Nav2 collision checking retained; not whole mission timing','measurement_note':'Odometry pose uses body origin; displacement must use TF base_control to avoid the 0.14m frame offset','records':records},indent=2));print(json.dumps(records))
