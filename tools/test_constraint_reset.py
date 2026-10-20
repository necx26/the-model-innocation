#!/usr/bin/env python3
"""Gazebo fault injection: an attachment unknown to the mission controller."""
import argparse
import copy
import json
import math
import time
import urllib.request
from pathlib import Path
import rclpy
from gazebo_msgs.msg import LinkStates,EntityState
from gazebo_msgs.srv import SetEntityState
from linkattacher_msgs.srv import AttachLink,DetachLink


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--inject-stale-grasp',action='store_true',required=True)
    parser.parse_args()
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def api(path,post=False):
        request=urllib.request.Request('http://127.0.0.1:8765'+path,data=b'' if post else None)
        with opener.open(request,timeout=5) as response:return json.load(response)
    state=api('/api/state')
    if state.get('busy') or state['phase']!='就绪':raise RuntimeError('只允许在空闲就绪状态做故障注入')
    rclpy.init();node=rclpy.create_node('stale_grasp_reset_probe');links={}
    sub=node.create_subscription(LinkStates,'/demo/link_states_demo',lambda m:links.update(zip(m.name,m.pose)),10)
    move=node.create_client(SetEntityState,'/demo/set_entity_state')
    attach=node.create_client(AttachLink,'/ATTACHLINK');detach=node.create_client(DetachLink,'/DETACHLINK')
    def call(client,request):
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('service unavailable')
        future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=6)
        if not future.done():raise TimeoutError('service timed out')
        return future.result()
    report={'purpose':'fault injection only, not an exam','cargo':'blue_cube_1','samples':[]}
    try:
        deadline=time.monotonic()+5
        while 'six_arm::link6' not in links and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        pose=copy.deepcopy(links['six_arm::link6']);pose.position.x+=.09
        pose.orientation.x=pose.orientation.y=pose.orientation.z=0.;pose.orientation.w=1.
        entity=EntityState(name='blue_cube_1',reference_frame='world',pose=pose)
        if not call(move,SetEntityState.Request(state=entity)).success:raise RuntimeError('injection placement failed')
        result=call(attach,AttachLink.Request(model1_name='six_arm',link1_name='link6',model2_name='blue_cube_1',link2_name='link'))
        report['injected_attachment']=result.success
        if not result.success:raise RuntimeError(result.message)
        # The controller did not create this constraint and has held_model=None.
        api('/api/reset?mode=fixed',True);deadline=time.monotonic()+35
        while time.monotonic()<deadline:
            state=api('/api/state')
            if not state.get('busy'):break
            time.sleep(.25)
        if state['phase']!='就绪':raise RuntimeError(state['status'])
        for _ in range(10):
            state=api('/api/state');report['samples'].append(state['robot']);time.sleep(.2)
        report['passed']=all(p and math.hypot(*p[:2])<.2 for p in report['samples'])
        report['phase']=state['phase'];report['healthy']=state['healthy']
        if not report['passed']:raise RuntimeError('post-reset robot escaped spawn')
    except Exception as error:
        report.update(passed=False,error=str(error));raise
    finally:
        call(detach,DetachLink.Request(model1_name='six_arm',link1_name='link6',model2_name='blue_cube_1',link2_name='link'))
        path=Path(__file__).resolve().parents[1]/'verification/stale-grasp-reset-20261020.json'
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2));print(str(path),report.get('passed'))
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
