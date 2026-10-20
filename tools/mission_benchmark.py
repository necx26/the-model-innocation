#!/usr/bin/env python3
"""Supervise real five-minute exams and save evidence after every round."""
import argparse
import hashlib
import json
import math
import time
import urllib.request
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OP=urllib.request.build_opener(urllib.request.ProxyHandler({}))


def api(path, value=None, post=False):
    data=json.dumps(value).encode() if value is not None else b'' if post else None
    request=urllib.request.Request('http://127.0.0.1:8765'+path,data=data,
                                   headers={'Content-Type':'application/json'})
    with OP.open(request,timeout=8) as response:return json.load(response)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--name',default=time.strftime('%Y%m%d-%H%M%S'))
    parser.add_argument('--rounds',type=int,default=1)
    parser.add_argument('--rule',type=int,default=1)
    parser.add_argument('--first',choices=['red','blue'],default='red')
    parser.add_argument('--seed',type=int)
    parser.add_argument('--layout',choices=['fixed','easy','distributed'],default='fixed')
    parser.add_argument('--replay-question',type=Path,help='test-only saved generator input; requires official input mode')
    args=parser.parse_args()
    try:runtime_mode=json.loads((ROOT/'.runtime/run-mode.json').read_text())['mode']
    except (OSError,ValueError,KeyError):runtime_mode='unverified runtime mode'
    directory=ROOT/'verification'/'mission_runs'/args.name
    directory.mkdir(parents=True,exist_ok=True)
    results=[]
    for number in range(args.rounds):
        path='/api/reset?mode='+args.layout if args.seed is None else '/api/reset?mode=distributed&seed='+str(args.seed+number)
        api(path,post=True)
        deadline=time.monotonic()+120
        while True:
            state=api('/api/state')
            if state['phase']=='重置失败':raise RuntimeError(state['status'])
            if state['phase']=='就绪' and not state.get('busy'):break
            if time.monotonic()>deadline:raise TimeoutError('场景准备超时')
            time.sleep(.5)
        api('/api/exam-config',{'rule':args.rule,'first':args.first,'question_mode':'competition'})
        if args.replay_question:
            if state.get('task_mode')!='official':raise RuntimeError('题目回放仅供回归测试，请先使用 YZBOT_TASK_MODE=official 启动')
            replay=json.loads(args.replay_question.read_text())
            if replay['context'].get('rule')!=args.rule or replay['context'].get('first')!=args.first:
                raise ValueError('回放题目的规则/顺序与测试参数不一致')
            api('/api/task',replay)
        initial_cubes=state.get('cubes',[])
        if not api('/api/start',post=True).get('started'):raise RuntimeError('考试没有开始')
        samples=[];phase_seconds={};phase_started=time.monotonic();last_phase=None
        started=time.monotonic();printed=0;hashes={};max_ages={};errors=[]
        with (directory/f'round-{number+1}.jsonl').open('w') as journal:
            while True:
                state=api('/api/state');now=time.monotonic()
                sample={k:state.get(k) for k in ['elapsed','remaining','phase','status','picked','placed','robot','camera_age','cameras','navigation_health','navigation_feedback','health_issues','gripper','task','known_targets']}
                samples.append(sample);journal.write(json.dumps(sample,ensure_ascii=False)+'\n');journal.flush()
                if state['phase']!=last_phase:
                    if last_phase:phase_seconds[last_phase]=phase_seconds.get(last_phase,0)+now-phase_started
                    last_phase=state['phase'];phase_started=now
                for name,value in state.get('navigation_health',{}).items():max_ages[name]=max(max_ages.get(name,0),value['pose_age'])
                if now-printed>=10:
                    print(f"round {number+1} {state['elapsed']:.1f}s {state['placed']}/{state['total']} {state['phase']} {state.get('robot')}",flush=True);printed=now
                if not state.get('running') and not state.get('busy'):break
                if now-started>335:
                    api('/api/stop',post=True);errors.append('supervisor wall-time limit');break
                time.sleep(1)
        phase_seconds[last_phase]=phase_seconds.get(last_phase,0)+time.monotonic()-phase_started
        physical=[]
        cubes={c['name']:c for c in state.get('cubes',[])}
        for result in state.get('result',[]):
            cube=cubes.get(result['model'],{});zx,zy=state['zones'][result['zone']]
            physical.append(bool(result.get('verified') and abs(cube.get('x',999)-zx)<.5 and abs(cube.get('y',999)-zy)<.25 and cube.get('z',999)<.16))
        passed=state['phase']=='完成' and state['placed']==state['total']==5 and len(physical)==5 and all(physical) and state['elapsed']<300
        # Independent test-only association, never fed back into perception,
        # search, navigation or experience training. Moved objects need review.
        tracks={}
        for sample in samples:
            for target in sample.get('known_targets',[]) or []:
                if target.get('hits',0)>=2:
                    previous=tracks.get(target['id'])
                    if previous is None or target['hits']>=previous['hits']:tracks[target['id']]=target
        camera_audit=[]
        for target in tracks.values():
            choices=[cube for cube in initial_cubes if cube['name'].startswith(target['color']+'_cube')]
            if not choices:continue
            nearest=min(choices,key=lambda cube:math.hypot(cube['x']-target['x'],cube['y']-target['y']))
            error=math.hypot(nearest['x']-target['x'],nearest['y']-target['y'])
            camera_audit.append(dict(target_id=target['id'],color=target['color'],hits=target['hits'],
                                     nearest_initial_model=nearest['name'],error_m=round(error,3),
                                     review_required=error>.30))
        report={'passed':passed,'environment':'ROS Humble: '+runtime_mode,'round':number+1,'elapsed':state['elapsed'],
                'question_source':'saved generator question replay (regression only)' if args.replay_question else 'live supplied generator',
                'remaining':state['remaining'],'task':state.get('task'),'physical_checks':physical,
                'phase_seconds':{k:round(v,2) for k,v in phase_seconds.items()},'max_navigation_pose_age':max_ages,
                'errors':errors,'initial_cubes':initial_cubes,
                'camera_track_audit':{'purpose':'test-only; large error may indicate motion or false detection','tracks':camera_audit},
                'final_state':state,'samples':samples}
        (directory/f'round-{number+1}.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        results.append({k:report[k] for k in ['round','passed','elapsed','remaining','task','physical_checks','phase_seconds','max_navigation_pose_age']})
        (directory/'summary.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
        from update_validation_report import update
        update()
        print(json.dumps(results[-1],ensure_ascii=False),flush=True)
        if not passed:raise SystemExit(1)
    print(str(directory),flush=True)


if __name__=='__main__':main()
