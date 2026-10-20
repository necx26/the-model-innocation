#!/usr/bin/env python3
"""Untimed map-grid survey using live camera tracks, never simulator cargo poses."""
import argparse
import json
import math
import signal
import time
import urllib.request
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer,TransformListener
from mission_strategy import MissionMap,wrap
from search_experience import SearchExperience

ROOT=Path(__file__).resolve().parents[1]
OP=urllib.request.build_opener(urllib.request.ProxyHandler({}))


def api(path,post=False):
    request=urllib.request.Request('http://127.0.0.1:8765'+path,data=b'' if post else None)
    with OP.open(request,timeout=5) as response:return json.load(response)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--side',choices=['all','east','west'],default='all')
    p.add_argument('--seconds',type=float,default=300)
    args=p.parse_args()
    if api('/api/state').get('busy'):raise RuntimeError('先停止当前作业再采集经验')
    api('/api/reset?mode=fixed',True)
    while True:
        state=api('/api/state')
        if state['phase']=='重置失败':raise RuntimeError(state['status'])
        if not state['busy'] and state['phase']=='就绪':break
        time.sleep(.3)
    if not api('/api/observe?enabled=1',True)['accepted']:raise RuntimeError('未获得视觉采集控制权')
    rclpy.init();node=Node('visual_experience_survey',parameter_overrides=[Parameter('use_sim_time',value=True)])
    user_stop=False
    def interrupt(*_):
        nonlocal user_stop
        user_stop=True
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    buffer=Buffer(node=node);listener=TransformListener(buffer,node)
    nav=ActionClient(node,NavigateToPose,'/navigate_to_pose')
    velocity=node.create_publisher(Twist,'/cmd_vel_nav',10)
    map_yaml=ROOT/'ros_ws/src/yzbot/bot_navigation/maps/map.yaml'
    grid=MissionMap(map_yaml)
    experience=SearchExperience(grid,map_yaml,ROOT/'data/search_experience','fixed_distributed')
    started=time.monotonic();last_poll=0.;state=api('/api/state');handle=None
    report={'purpose':'untimed visual survey, not competition acceptance','side':args.side,'views':[]}
    def tick():
        nonlocal state,last_poll
        if user_stop:raise InterruptedError('用户停止视觉采集')
        rclpy.spin_once(node,timeout_sec=.02)
        now=time.monotonic()
        if now-last_poll>.15:
            state=api('/api/state');last_poll=now
            if state.get('stop_requested'):raise InterruptedError('视觉采集已停止')
            if not state.get('healthy'):raise RuntimeError('采集健康检查失败: '+str(state.get('health_issues')))
        if now-started>args.seconds:raise TimeoutError('达到本次视觉采集时间上限')
    def pose():
        t=buffer.lookup_transform('odom','base_control',Time()).transform;q=t.rotation
        return t.translation.x,t.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def wait(future,seconds):
        deadline=time.monotonic()+seconds
        while not future.done():
            tick()
            if time.monotonic()>deadline:raise TimeoutError('观察点导航等待超时')
        return future.result()
    def inventory():
        landmarks=[]
        for index,color in enumerate(('red','blue')):
            heat=experience.heat[index].copy()
            for _ in range(16):
                row,col=np.unravel_index(np.argmax(heat),heat.shape)
                if heat[row,col]<.2:break
                landmarks.append((color,*grid.xy((row,col))))
                rr,cc=np.ogrid[:heat.shape[0],:heat.shape[1]]
                heat[(rr-row)**2+(cc-col)**2<49]=0
        for t in state.get('known_targets',[]):
            if t['hits']>=2 and not any(c==t['color'] and math.hypot(t['x']-x,t['y']-y)<.6 for c,x,y in landmarks):
                landmarks.append((t['color'],t['x'],t['y']))
        return {color:sum(c==color for c,x,y in landmarks) for color in ('red','blue')}
    try:
        end=time.monotonic()+2
        while time.monotonic()<end:tick()
        xmin,xmax,ymin,ymax=grid.bounds
        viewpoints=[]
        for column in range(3):
            for row in range(4):
                x=xmin+(column+.5)*(xmax-xmin)/3;y=ymin+(row+.5)*(ymax-ymin)/4
                if args.side=='east' and column!=2:continue
                if args.side=='west' and column!=0:continue
                candidates=[v['point'] for v in grid.views
                            if xmin+column*(xmax-xmin)/3<=v['point'][0]<=xmin+(column+1)*(xmax-xmin)/3
                            and ymin+row*(ymax-ymin)/4<=v['point'][1]<=ymin+(row+1)*(ymax-ymin)/4]
                if candidates:viewpoints.append(min(candidates,key=lambda q:math.dist(q,(x,y))))
        while viewpoints and not all(n>=5 for n in inventory().values()):
            current=pose();field=grid.distances(current)
            point=min(viewpoints,key=lambda q:grid.distance(field,q));viewpoints.remove(point)
            print('map survey view',point,'inventory',inventory(),flush=True)
            goal=NavigateToPose.Goal();goal.pose.header.frame_id='odom'
            goal.pose.pose.position.x=float(point[0]);goal.pose.pose.position.y=float(point[1]);goal.pose.pose.orientation.w=1.
            handle=wait(nav.send_goal_async(goal),8)
            if not handle.accepted:continue
            result=wait(handle.get_result_async(),50);handle=None
            record={'point':point,'navigation_status':result.status,'seconds':round(time.monotonic()-started,2)}
            report['views'].append(record)
            if result.status!=4:continue
            for yaw in (0.,math.pi/2,math.pi,-math.pi/2):
                current=pose()
                if any(math.dist(current[:2],(t['x'],t['y']))<.5 for t in state.get('known_targets',[]) if t['status']=='available'):break
                turn_end=time.monotonic()+8
                while time.monotonic()<turn_end:
                    tick();error=wrap(yaw-pose()[2])
                    if abs(error)<.10:break
                    msg=Twist();msg.angular.z=max(-.55,min(.55,1.5*error));velocity.publish(msg)
                velocity.publish(Twist())
                settle=time.monotonic()+.6
                while time.monotonic()<settle:tick()
                if all(n>=5 for n in inventory().values()):break
        report['inventory_prior_plus_live']=inventory()
    except (TimeoutError,InterruptedError,RuntimeError) as error:report['stopped_reason']=str(error)
    finally:
        if handle is not None:
            try:rclpy.spin_until_future_complete(node,handle.cancel_goal_async(),timeout_sec=3)
            except Exception:pass
        for _ in range(5):velocity.publish(Twist());rclpy.spin_once(node,timeout_sec=.05)
        targets=state.get('known_targets',[])
        report.update(elapsed=round(time.monotonic()-started,2),camera_tracks=targets)
        report['learned']=experience.learn(targets,'survey:'+time.strftime('%Y%m%d-%H%M%S'))
        path=ROOT/'verification'/('visual-survey-'+time.strftime('%Y%m%d-%H%M%S')+'.json')
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2))
        api('/api/observe?enabled=0',True)
        if not user_stop and not state.get('stop_requested'):api('/api/reset?mode=fixed',True)
        print(str(path),json.dumps({k:v for k,v in report.items() if k not in ('camera_tracks','views')},ensure_ascii=False),flush=True)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
