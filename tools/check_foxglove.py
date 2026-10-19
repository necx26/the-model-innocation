#!/usr/bin/env python3
"""Verify real WebSocket topics; optionally trigger the same empty-message button."""
import argparse
import asyncio
import json
import struct
import time
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'vendor/python'))
import websockets


async def main(start_exam):
    wanted={'/competition/question','/competition/red_count','/competition/blue_count',
            '/competition/annotated_image','/camera/image_raw','/competition/work_phase','/map','/competition/path'}
    observed=set();channels={};subscribed=set()
    async with websockets.connect('ws://127.0.0.1:8766',subprotocols=['foxglove.websocket.v1'],max_size=32*1024*1024) as ws:
        end=time.monotonic()+20;triggered=False
        while time.monotonic()<end:
            item=await asyncio.wait_for(ws.recv(),timeout=10)
            if isinstance(item,str):
                data=json.loads(item)
                if data.get('op')=='advertise':
                    for c in data['channels']:channels[c['id']]=c
                    subs=[{'id':id,'channelId':id} for id,c in channels.items() if c['topic'] in wanted and id not in subscribed]
                    if subs:
                        await ws.send(json.dumps(dict(op='subscribe',subscriptions=subs)));subscribed.update(s['id'] for s in subs)
                elif data.get('op')=='status' and data.get('level',0)>=2:raise RuntimeError(data)
            elif item and item[0]==1:
                id=struct.unpack_from('<I',item,1)[0]
                if id in channels:observed.add(channels[id]['topic'])
            needed=wanted-{'/competition/path'} # no path is expected before navigation starts
            if needed<=observed:
                if start_exam and not triggered:
                    await ws.send(json.dumps(dict(op='advertise',channels=[dict(id=999,topic='/competition/start_exam',
                        encoding='cdr',schemaName='std_msgs/msg/Empty')])) )
                    await asyncio.sleep(.5)
                    await ws.send(bytes([1])+struct.pack('<I',999)+b'\x00\x01\x00\x00\x00')
                    triggered=True
                print(json.dumps(dict(received_topics=sorted(observed),start_button_sent=triggered),ensure_ascii=False));return
        raise RuntimeError('Foxglove缺少实时数据：'+str(wanted-observed))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--start-exam',action='store_true');args=p.parse_args()
    asyncio.run(main(args.start_exam))
