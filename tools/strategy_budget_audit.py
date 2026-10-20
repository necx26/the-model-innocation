#!/usr/bin/env python3
"""Offline benchmark only. Never supplies simulator positions to the controller.

The seeded fixture is exposed ONLY to estimate an optimistic delivery budget.
Coverage estimates use map LOS and require separate real-camera validation.
"""
import sys,json,math,argparse
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'ros_ws'))
from mission_strategy import MissionMap,plan_deliveries,CRUISE_SPEED
from scene_config import ZONES
p=argparse.ArgumentParser();p.add_argument('--seed',type=int,default=3069713103);args=p.parse_args()
f=MissionMap(ROOT/'ros_ws/src/yzbot/bot_navigation/maps/map.yaml')
items=f.scattered_layout(args.seed)['layout'];targets=[dict(id=i+1,color=t['color'],x=t['x'],y=t['y']) for i,t in enumerate(items)]
route=plan_deliveries(f,(.14,0,0),targets,['red']*4+['blue'],{'red':ZONES['A'],'blue':ZONES['C']})
result={'purpose':'offline diagnostic only; fixture coordinates never sent to runtime planner','seed':args.seed,'task':{'red':4,'blue':1,'first':'red','red_zone':'A','blue_zone':'C'},'cruise_mps':CRUISE_SPEED,'all_targets_known_delivery_estimate_s':route[0]['known_plan_seconds'] if route else None,'estimate_assumptions':'current map-weighted route model, 17 seconds service per object; optimistic, not a guaranteed execution time','delivery_ids':[r['target']['id'] for r in route]}
print(json.dumps(result,ensure_ascii=False),flush=True)
# Greedy continuous coverage tour: no depot returns between observations.
seen=np.zeros(len(f.search_points),bool);current=(.14,0);tour=[];travel=0.
while not seen.all() and len(tour)<64:
 field=f.distances(current);best=None
 for i,view in enumerate(f.views):
  gain=int(np.count_nonzero(~seen[view['visible']]))
  if not gain:continue
  d=f.distance(field,view['point'])
  if not math.isfinite(d):continue
  score=gain/(d/CRUISE_SPEED+7.)
  if best is None or score>best[0]:best=(score,i,gain,d)
 if best is None:break
 _,i,gain,d=best;view=f.views[i];seen[view['visible']]=True;travel+=d;current=view['point']
 tour.append({'point':list(current),'new_samples':gain,'travel_cost_m':round(d,2)})
result.update(coverage_model='map visibility within 3.5 m; assumes 7 s full sweep at each view; does not prove real image coverage',coverage_sample_fraction=float(np.mean(seen)),coverage_view_count=len(tour),coverage_travel_cost_m=round(travel,1),coverage_estimate_s=round(travel/CRUISE_SPEED+7.*len(tour),1),coverage_tour=tour)
result['conclusion']='Compare predicted coverage plus delivery time with 300 s before implementing explore-all-first; estimates are diagnostic, not acceptance.'
out=ROOT/'verification/strategy-budget-audit-20261019.json';out.write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps({k:v for k,v in result.items() if k!='coverage_tour'},ensure_ascii=False,indent=2),flush=True)
