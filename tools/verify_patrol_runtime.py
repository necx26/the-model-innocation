#!/usr/bin/env python3
"""Observe one full obstacle cycle against the actual loaded terrain pose."""
import json,math,time
from pathlib import Path
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from patrol_safety import terrain_boxes,boxes_clearance,CAR_FRONT,CAR_REAR,CAR_HALF_WIDTH,WALL_MARGIN
from scene_config import SCENE,PATROLS
root=Path('/opt/yzbot');world=root/'ros_ws/src/yzbot/mybot_description/worlds'/SCENE['terrain_world']
boxes=terrain_boxes(world);models={};records={p['model']:dict(samples=0,min_margin=999.,axis_min=[999.,999.],axis_max=[-999.,-999.]) for p in PATROLS}
rclpy.init();node=Node('patrol_runtime_audit')
node.create_subscription(ModelStates,'/demo/model_states_demo',lambda m:models.update(zip(m.name,m.pose)),10)
try:
    end=time.monotonic()+10
    while not all(n in models for n in ['officeroom',*records]) and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    saved=ET.parse(world).getroot().find("world/state/model[@name='officeroom']/pose")
    expected=list(map(float,saved.text.split()))
    actual=models['officeroom'].position
    pose_error=math.dist([actual.x,actual.y,actual.z],expected[:3])
    if pose_error>1e-3:raise RuntimeError('运行地形位置与巡逻检查的保存状态不同')
    duration=max(p['period'] for p in PATROLS)+2;end=time.monotonic()+duration
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.1)
        for name,record in records.items():
            pose=models[name];q=pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            offset=(CAR_FRONT-CAR_REAR)/2
            car=dict(pose=(pose.position.x+offset*math.cos(yaw),pose.position.y+offset*math.sin(yaw),0.,yaw),size=(CAR_FRONT+CAR_REAR,2*CAR_HALF_WIDTH,.72))
            gap=min(boxes_clearance(car,box) for box in boxes);record['samples']+=1;record['min_margin']=min(record['min_margin'],gap)
            for i,value in enumerate((pose.position.x,pose.position.y)):
                record['axis_min'][i]=min(record['axis_min'][i],value);record['axis_max'][i]=max(record['axis_max'][i],value)
    for p in PATROLS:
        record=records[p['model']];record['observed_travel_span']=math.dist(record['axis_min'],record['axis_max'])
        record['passed']=record['min_margin']>=WALL_MARGIN and record['observed_travel_span']>=math.dist(p['start'],p['end'])-.03
    result=dict(duration_seconds=duration,terrain_pose_error=pose_error,records=records,passed=all(r['passed'] for r in records.values()))
    (root/'verification/patrol-runtime.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    if not result['passed']:raise RuntimeError('巡逻实测未通过')
finally:node.destroy_node();rclpy.shutdown()
