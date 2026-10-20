#!/usr/bin/env python3
"""Fail startup if an obstacle's complete swept path intersects terrain."""
import json,sys,math
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'ros_ws'))
from scene_config import SCENE,PATROLS
from patrol_safety import patrol_reports
world=ROOT/'ros_ws/src/yzbot/mybot_description/worlds'/SCENE['terrain_world']
records=patrol_reports(PATROLS,world);tree=ET.parse(world).getroot().find('world')
for patrol in PATROLS:
    model=tree.find("model[@name='"+patrol['model']+"']")
    plugin=model.find("plugin[@name='competition_patrol']")
    for key in ('start','end'):
        loaded=list(map(float,plugin.findtext(key).split()))
        if math.dist(loaded,patrol[key])>1e-6:raise SystemExit('障碍车场景参数未同步，请执行 python3 tools/configure_scene.py')
    if abs(float(plugin.findtext('period'))-patrol['period'])>1e-6:raise SystemExit('障碍车周期参数未同步')
print(json.dumps(records,ensure_ascii=False,indent=2))
