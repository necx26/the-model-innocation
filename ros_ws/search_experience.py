"""Map-specific observation priors learned only from earlier camera tracks.

These priors propose places to LOOK. They never create an ObjectMemory target,
provide attachment identity, or authorize a grasp without current images.
"""
import hashlib
import json
import math
from pathlib import Path
import numpy as np


class SearchExperience:
    def __init__(self, grid, map_yaml, directory, scenario):
        self.grid=grid
        meta=Path(map_yaml)
        import yaml
        image=meta.parent/yaml.safe_load(meta.read_text())['image']
        self.map_key=hashlib.sha256(meta.read_bytes()+image.read_bytes()).hexdigest()[:16]
        self.path=Path(directory)/(self.map_key+'-'+scenario+'.npz')
        self.heat=np.zeros((2,*grid.free.shape),dtype=np.float32)
        self.runs=[]
        if self.path.exists():
            with np.load(self.path,allow_pickle=False) as data:
                if ('policy_version' in data and int(data['policy_version'])==2
                        and data['heat'].shape==self.heat.shape):
                    self.heat=data['heat'].copy()
                    self.runs=json.loads(str(data['runs']))

    def learn(self, targets, run_id):
        if run_id in self.runs:return 0
        added=0;recorded=[]
        for target in targets:
            if target.get('hits',0)<2 or target.get('status') not in ('available','picked','delivered'):
                continue
            # A transient two-frame candidate can be useful in the current
            # round, but must not become a permanent observation hotspot.
            if target.get('status')=='available' and target['hits']<5:continue
            x,y=float(target['x']),float(target['y']);color=target['color']
            if any(c==color and math.hypot(x-px,y-py)<.45 for c,px,py in recorded):continue
            row,col=self.grid.cell((x,y))
            if not self.grid.inside((row,col)):continue
            index=0 if color=='red' else 1
            for dr in range(-5,6):
                for dc in range(-5,6):
                    r,c=row+dr,col+dc
                    if self.grid.inside((r,c)) and self.grid.free[r,c]:
                        self.heat[index,r,c]+=math.exp(-(dr*dr+dc*dc)/8.)
            recorded.append((color,x,y));added+=1
        self.runs.append(run_id)
        self.path.parent.mkdir(parents=True,exist_ok=True)
        temporary=self.path.with_suffix('.tmp')
        with temporary.open('wb') as stream:
            np.savez_compressed(stream,heat=self.heat,runs=json.dumps(self.runs),policy_version=2)
        temporary.replace(self.path)
        return added

    def observation(self, current, color, attempted, delivery):
        remaining=self.heat[0 if color=='red' else 1].copy()
        distances=self.grid.distances(current);back=self.grid.distances(delivery)
        best=None
        for _ in range(16):
            row,col=np.unravel_index(np.argmax(remaining),remaining.shape)
            confidence=float(remaining[row,col])
            if confidence<.2:break
            centre=self.grid.xy((row,col))
            rr,cc=np.ogrid[:remaining.shape[0],:remaining.shape[1]]
            remaining[(rr-row)**2+(cc-col)**2<49]=0
            if any(math.dist(centre,point)<.85 for point in attempted):continue
            headings=list(np.arange(0.,2*math.pi,math.pi/4))
            headings.append(math.atan2(centre[1]-current[1],centre[0]-current[0]))
            for yaw in headings:
                point=(centre[0]-.95*math.cos(yaw),centre[1]-.95*math.sin(yaw))
                if not self.grid.safe(point) or not self.grid.line_free(point,centre):continue
                travel=self.grid.distance(distances,point)
                home=self.grid.distance(back,point)
                if not math.isfinite(travel+home):continue
                cost=travel+.35*home+1.
                score=min(1.,confidence)/cost
                if best is None or score>best['score']:
                    best={'score':score,'centre':centre,'pose':(*point,float(yaw)),
                          'confidence':round(confidence,3),'travel_m':round(travel,2)}
        return best
