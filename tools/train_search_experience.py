#!/usr/bin/env python3
"""Import confirmed camera tracks from saved test reports, never cube truth."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros_ws'))
from mission_strategy import MissionMap
from search_experience import SearchExperience

parser=argparse.ArgumentParser()
parser.add_argument('reports',nargs='*',type=Path)
parser.add_argument('--events',type=Path,help='import camera-confirmation events from fixed-layout test blocks')
parser.add_argument('--survey',type=Path,help='import an untimed fixed-layout camera survey report')
parser.add_argument('--output-directory',type=Path,default=ROOT/'data/search_experience')
parser.add_argument('--layout-seed',type=int,default=4)
args=parser.parse_args()
if not args.reports and args.events is None and args.survey is None:parser.error('需要测试报告、--survey 或 --events 日志')
map_yaml=ROOT/'ros_ws/src/yzbot/bot_navigation/maps/map.yaml'
grid=MissionMap(map_yaml)
manifest=[]
for path in args.reports:
    report=json.loads(path.read_text())
    state=report.get('final_state',report.get('final',report))
    scenario=state.get('layout_mode')
    if scenario not in ('fixed_distributed','fixed_visible','distributed_office'):
        raise ValueError('报告缺少已验证的布局类型: '+str(path))
    experience=SearchExperience(grid,map_yaml,args.output_directory,scenario)
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    added=experience.learn(state.get('known_targets',[]),'report:'+digest)
    manifest.append({'report':str(path),'sha256':digest,'scenario':scenario,
                     'source':'known_targets from camera tracks only','added':added,
                     'map_key':experience.map_key})
if args.survey:
    survey=json.loads(args.survey.read_text())
    if survey.get('purpose')!='untimed visual survey, not competition acceptance':
        raise ValueError('不是固定布局视觉采集记录')
    experience=SearchExperience(grid,map_yaml,args.output_directory,'fixed_distributed')
    digest=hashlib.sha256(args.survey.read_bytes()).hexdigest()
    added=experience.learn(survey['camera_tracks'],'survey-report:'+digest)
    manifest.append({'survey':str(args.survey),'sha256':digest,'scenario':'fixed_distributed',
                     'source':'camera_tracks only; no ground truth input','added':added,'map_key':experience.map_key})
if args.events:
    data=args.events.read_bytes();active=False;groups=[]
    for line in data.decode().splitlines():
        try:event=json.loads(line)
        except ValueError:continue
        text=event.get('text','')
        if '场景已复位' in text:
            active='固定分区' in text and event.get('seed')==args.layout_seed
        if not (active and text=='相机发现并记住目标' and event.get('source')=='onboard_camera'):continue
        point=event['position'];color=event['color']
        choices=[g for g in groups if g['color']==color and math.dist(g['points'][-1],point)<.45]
        if choices:choices[0]['points'].append(point)
        else:groups.append({'color':color,'points':[point]})
    import numpy as np
    targets=[]
    for group in groups:
        x,y=np.median(group['points'],axis=0)
        targets.append(dict(color=group['color'],x=float(x),y=float(y),hits=2*len(group['points']),status='available'))
    experience=SearchExperience(grid,map_yaml,args.output_directory,'fixed_distributed')
    digest=hashlib.sha256(data).hexdigest()
    added=experience.learn(targets,'events:'+digest)
    manifest.append({'events':str(args.events),'sha256':digest,'scenario':'fixed_distributed',
                     'layout_seed_filter':args.layout_seed,'source':'onboard_camera confirmation events only; each event requires two frames',
                     'added':added,'map_key':experience.map_key})
out=ROOT/'verification/search-experience-training.json'
out.write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
print(json.dumps(manifest,ensure_ascii=False,indent=2))
