#!/usr/bin/env python3
"""Update shared world configuration and predictive map, preserving geometry."""
import argparse
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros_ws'))
from scene_config import SCENE, ZONES, PATROLS


def apply_world():
    path = ROOT/'ros_ws/src/yzbot/mybot_description/worlds'/SCENE.get('terrain_world', 'competition.world')
    tree = ET.parse(path); world = tree.getroot().find('world')
    models = {m.get('name'): m for m in world.findall('model')}
    # Encode in Gazebo's process, before DDS: do not transfer 691 KB raw frames
    # through the USB OS's small default UDP receive buffers.
    for view in ('overview', 'oblique'):
        sensor = models['competition_camera_'+view].find('link/sensor')
        plugin = sensor.find("plugin[@name='"+view+"_compressed_camera']")
        if plugin is None:
            plugin = ET.SubElement(sensor, 'plugin', name=view+'_compressed_camera',
                                   filename='libcompetition_camera_stream.so')
            ET.SubElement(plugin, 'topic_name').text='/fixed_'+view+'/image_raw/compressed'
            ET.SubElement(plugin, 'frame_name').text='fixed_'+view+'_camera'
    world.find("plugin[@name='competition_link_attacher']").set('filename','libgazebo_link_attacher.so')
    # Spawn the competition scene before the controller completes readiness.
    # Runtime search never reads this scene-generation data.
    from mission_strategy import MissionMap
    field=MissionMap(ROOT/'ros_ws/src/yzbot/bot_navigation/maps/map.yaml')
    initial=field.scattered_layout(4)
    for item in initial['layout']:
        models[item['model']].find('pose').text=f"{item['x']} {item['y']} .025 0 0 0"
    for name, (x,y) in ZONES.items():
        models['zone_'+name.lower()].find('pose').text = f'{x} {y} 0.01 0 0 0'
    for patrol in PATROLS:
        assert math.dist(patrol['start'], patrol['end']) >= 2., patrol
        model = models[patrol['model']]
        plugin = model.find("plugin[@name='competition_patrol']")
        plugin.set('filename', 'libcompetition_patrol.so')
        for key in ('start','end'):
            plugin.find(key).text = ' '.join(map(str,patrol[key]))
        plugin.find('period').text = str(patrol['period'])
        model.find('pose').text = ' '.join(map(str,patrol['start']))+' 0 0 1.57079632679'
        old = model.find("link[@name='picture_placard']")
        if old is not None: model.remove(old)
        # Native car body/wheels remain unchanged. The board is carried by it.
        model.append(ET.fromstring('''<link name="picture_placard">
          <pose>0 0 0 0 0 0</pose>
          <visual name="support"><pose>0 0 .25 0 0 0</pose>
            <geometry><cylinder><radius>.008</radius><length>.22</length></cylinder></geometry>
            <material><ambient>.12 .12 .12 1</ambient><diffuse>.12 .12 .12 1</diffuse></material></visual>
          <visual name="picture"><pose>0 0 .49 0 0 0</pose>
            <geometry><box><size>.012 .26 .394</size></box></geometry>
            <material><script><uri>model://yzbot_signs/materials/scripts</uri>
              <uri>model://yzbot_signs/materials/textures</uri><name>Yzbot/UserPlacard</name></script></material>
            <cast_shadows>false</cast_shadows></visual>
          <collision name="board"><pose>0 0 .49 0 0 0</pose>
            <geometry><box><size>.012 .26 .394</size></box></geometry></collision>
        </link>'''))
    ET.indent(tree, space='  ')
    tree.write(path, encoding='utf-8', xml_declaration=True)
    print('Updated world:',path)


def apply_map():
    import cv2
    import numpy as np
    import yaml
    root=ROOT/'ros_ws/src/yzbot/bot_navigation/maps'
    meta=yaml.safe_load((root/'map.yaml').read_text())
    original=cv2.imread(str(root/meta['image']),0)
    h,w=original.shape;res=meta['resolution'];ox,oy,_=meta['origin']
    raw=np.where(original>250,0,np.where(original<100,100,255)).astype('uint8')
    xs=np.arange(w)*res+ox;ys=(h-1-np.arange(h))*res+oy
    for p in PATROLS:
        a,b=p['start'],p['end']
        # Expanded lane envelopes; actual moving vehicles also enter laser layers.
        mask=(xs[None,:]>min(a[0],b[0])-.65)&(xs[None,:]<max(a[0],b[0])+.65)&(ys[:,None]>min(a[1],b[1])-.65)&(ys[:,None]<max(a[1],b[1])+.65)
        raw[mask&(original>250)]=60
    cv2.imwrite(str(root/'competition_safe_map.pgm'),raw)
    meta.update(image='competition_safe_map.pgm',mode='raw')
    (root/'competition_safe_map.yaml').write_text(yaml.safe_dump(meta))
    nav_path=root.parent/'param/competition_nav2.yaml'
    nav=yaml.safe_load(nav_path.read_text())
    nav['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel']=SCENE.get('cruise_speed_mps',.65)
    nav['velocity_smoother']['ros__parameters']['max_velocity'][0]=SCENE.get('cruise_speed_mps',.65)
    nav_path.write_text(yaml.safe_dump(nav,sort_keys=False))
    print('Updated patrol cost map and cruise speed')


if __name__ == '__main__':
    apply_world();apply_map()
