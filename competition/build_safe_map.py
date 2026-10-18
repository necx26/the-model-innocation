#!/usr/bin/env python3
"""Encode a predictive cost map without altering walls or unknown cells."""
from pathlib import Path
from PIL import Image,ImageDraw
import numpy as np
import yaml
root=Path(__file__).resolve().parents[1]/'ros_ws/src/yzbot/bot_navigation/maps'
im=Image.open(root/'map.pgm');original=np.array(im);w,h=im.size
mask=Image.new('L',im.size,0);draw=ImageDraw.Draw(mask)
for x,low,high in [(.5,-1.2,.2),(1.6,-.2,1.)]:
 draw.rectangle([round((x-.65+50)/.05),h-1-round((high+.65+50)/.05),round((x+.65+50)/.05),h-1-round((low-.65+50)/.05)],fill=255)
# Nav2 map_server raw mode: 0 free, 100 occupied, 255 unknown.
raw=np.where(original>250,0,np.where(original<100,100,255)).astype('uint8')
raw[(np.array(mask)>0)&(original>250)]=60
Image.fromarray(raw).save(root/'competition_safe_map.pgm')
meta=yaml.safe_load((root/'map.yaml').read_text());meta.update(image='competition_safe_map.pgm',mode='raw')
(root/'competition_safe_map.yaml').write_text(yaml.safe_dump(meta))
