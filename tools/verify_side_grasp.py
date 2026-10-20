#!/usr/bin/env python3
"""Gazebo-only side grasp diagnostic. Stop the HTTP manager first, then reset.

Fixture setup uses a simulator cube pose only to position the robot before
observation. All target coordinates supplied to approach/pick come from camera
frames. This diagnostic is not a timed competition acceptance run.
"""
import copy,json,math,threading,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from linkattacher_msgs.srv import DetachLink
from competition_node import Competition
from mission_strategy import ObjectMemory

def main():
    rclpy.init();app=Competition();thread=threading.Thread(target=rclpy.spin,args=(app,),daemon=True);thread.start()
    records=[];original=None;name='blue_cube_3'
    def place(entity,pose):
        request=EntityState(name=entity,reference_frame='world');request.pose=copy.deepcopy(pose)
        if not app.call(app.set_entity,SetEntityState.Request(state=request)).success:raise RuntimeError('fixture placement failed')
        time.sleep(.5)
    try:
        deadline=time.monotonic()+15
        while (name not in app.models or app.robot_pose() is None) and time.monotonic()<deadline:time.sleep(.1)
        original=copy.deepcopy(app.models[name]);original.position.z=.025
        for sign in (-1,1):
            app.cancel.clear();app.memory=ObjectMemory();app.used=set();app.busy=False
            app.state.update(picked=0,placed=0,result=[]);app.perception_paused=False
            app.grip(False)
            # Begin with a front camera observation, then turn the empty,
            # stowed chassis away. The cube remains in confirmed visual memory.
            robot=copy.deepcopy(original);robot.position.x-=.45;robot.position.z=0.
            robot.orientation.x=robot.orientation.y=robot.orientation.z=0.;robot.orientation.w=1.
            place('six_arm',robot);app.settle_base();time.sleep(.5)
            observations=[];frame_keys=[]
            for _ in range(4):
                values,_,stamp=app.vision(with_pose=True)
                key=(stamp.sec,stamp.nanosec)
                if key in frame_keys:raise RuntimeError('diagnostic camera frame did not advance')
                frame_keys.append(key)
                candidates=[d for d in values if d['color']=='blue' and math.dist((d['x'],d['y']),(original.position.x,original.position.y))<.15]
                if not candidates:raise RuntimeError('fixture cube not camera-confirmed')
                observations.append(min(candidates,key=lambda d:math.dist((d['x'],d['y']),(original.position.x,original.position.y))))
                time.sleep(.1)
            # model origin is .14 m behind base_control: .45 setup -> .31 gap.
            pose=app.robot_pose();app.align(*pose[:2],sign*math.radians(80),position_tolerance=.025,yaw_tolerance=.015)
            app.settle_base();before=app.robot_pose();d=observations[-1]
            target=dict(d,id=900+sign,hits=len(observations))
            selection=dict(target=target,grasp=before,pregrasp=before,view=before)
            if app.capture_preflight(target) is None:raise RuntimeError('side fixture outside collision-safe IK workspace')
            started=time.monotonic();entity,actual=app.approach_target(selection);app.pick_up(entity,actual,target['id'])
            after=app.robot_pose();cube=app.models[entity].position
            record=dict(direction='left' if sign<0 else 'right',camera_frames=len(observations),
                        target_source='camera',frame_timestamps=frame_keys,base_before=before,base_after=after,
                        base_translation=math.dist(before[:2],after[:2]),
                        base_yaw_change=abs((after[2]-before[2]+math.pi)%(2*math.pi)-math.pi),
                        held_model=app.held_model,lifted_z=cube.z,seconds=time.monotonic()-started,
                        metrics=copy.deepcopy(app.manipulation_metrics),events=copy.deepcopy(app.state['events']))
            record['passed']=record['base_translation']<.01 and record['base_yaw_change']<.02 and cube.z>.12 and app.held_model==name
            records.append(record)
            app.call(app.detach,DetachLink.Request(model1_name='six_arm',link1_name='link6',model2_name=entity,link2_name='link'))
            app.held_model=None;app.grip(False)
            pose=app.robot_pose();app.arm_xyz(pose[0]+.10*math.cos(pose[2]),pose[1]+.10*math.sin(pose[2]),.43)
            place(name,original)
    finally:
        app.halt_motion()
        if app.held_model:
            app.call(app.detach,DetachLink.Request(model1_name='six_arm',link1_name='link6',model2_name=app.held_model,link2_name='link'))
        if original is not None:place(name,original)
        out=Path('/opt/yzbot/verification/side-grasp-diagnostic.json');out.write_text(json.dumps(dict(purpose=__doc__,records=records),ensure_ascii=False,indent=2))
        rclpy.shutdown();thread.join(timeout=2);app.destroy_node()
    print(json.dumps([dict(direction=r['direction'],passed=r['passed'],seconds=r['seconds'],base_translation=r['base_translation'],base_yaw_change=r['base_yaw_change'],lifted_z=r['lifted_z']) for r in records],indent=2))
    if len(records)!=2 or not all(r['passed'] for r in records):raise RuntimeError('side grasp diagnostic failed')
if __name__=='__main__':main()
