#!/usr/bin/env python3
"""ROS 2 CDR bridge for the supplied Foxglove WebSocket v1 layout.

The 2026 SDK's v1 subprotocol is different from the legacy visualization
protocol. This bridge keeps local, offline viewers interoperable with Humble.
Only the two explicit Empty control topics permit client publication.
"""
import asyncio
import json
import re
import struct
import sys
import threading
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'vendor/python'))
import websockets
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from rosidl_runtime_py.utilities import get_message

TOPICS={
    '/competition/question':'std_msgs/msg/String', '/competition/order':'std_msgs/msg/String',
    '/competition/red_count':'std_msgs/msg/UInt32','/competition/blue_count':'std_msgs/msg/UInt32',
    '/competition/red_zone':'std_msgs/msg/String','/competition/blue_zone':'std_msgs/msg/String',
    '/competition/progress':'std_msgs/msg/UInt32','/competition/work_phase':'std_msgs/msg/String',
    '/competition/gripper_code':'std_msgs/msg/String','/competition/work_state':'std_msgs/msg/String',
    '/competition/gripper_state':'std_msgs/msg/String','/competition/status':'std_msgs/msg/String',
    '/competition/remaining':'std_msgs/msg/Float32','/competition/path':'nav_msgs/msg/Path',
    '/camera/image_raw':'sensor_msgs/msg/Image','/camera/camera_info':'sensor_msgs/msg/CameraInfo',
    '/competition/annotated_image':'sensor_msgs/msg/Image',
    '/fixed_overview/image_raw':'sensor_msgs/msg/Image','/fixed_oblique/image_raw':'sensor_msgs/msg/Image',
    '/map':'nav_msgs/msg/OccupancyGrid','/competition/navigation_map':'nav_msgs/msg/OccupancyGrid',
    '/slam/map':'nav_msgs/msg/OccupancyGrid','/scan':'sensor_msgs/msg/LaserScan',
    '/tf':'tf2_msgs/msg/TFMessage','/tf_static':'tf2_msgs/msg/TFMessage',
    '/robot_description':'std_msgs/msg/String','/joint_states':'sensor_msgs/msg/JointState',
    '/competition/start_exam':'std_msgs/msg/Empty','/competition/stop':'std_msgs/msg/Empty'}
CONTROLS={'/competition/start_exam','/competition/stop'}
LATCHED={'/tf_static','/robot_description','/map','/slam/map','/competition/navigation_map'}


def schema(type_name):
    seen=set();dependencies=[]
    def read(name):
        pkg,kind=name.split('/')[0],name.split('/')[-1]
        canonical=pkg+'/'+kind
        if canonical in seen:return ''
        seen.add(canonical)
        text=(Path(get_package_share_directory(pkg))/'msg'/(kind+'.msg')).read_text()
        for line in text.splitlines():
            line=line.split('#',1)[0].strip()
            if not line:continue
            field=line.split()[0].split('[',1)[0]
            if '/' in field:
                nested=read(field)
                if nested:dependencies.append(('MSG: '+field,nested))
        return text
    root=read(type_name)
    return root+''.join('\n'+'='*80+'\n'+name+'\n'+text for name,text in dependencies)


class Bridge(Node):
    def __init__(self,loop):
        super().__init__('competition_visualization_bridge',parameter_overrides=[Parameter('use_sim_time',value=True)],automatically_declare_parameters_from_overrides=True)
        self.loop=loop;self.channels={};self.peers={};self.latest={};self.controls={}
        for i,(topic,typename) in enumerate(TOPICS.items(),1):
            self.channels[i]=dict(id=i,topic=topic,encoding='cdr',schemaName=typename,schemaEncoding='ros2msg',schema=schema(typename))
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic in LATCHED else qos_profile_sensor_data
            self.create_subscription(get_message(typename),topic,lambda raw,id=i:self.received(id,raw),qos,raw=True)
        empty=get_message('std_msgs/msg/Empty');self.empty_type=empty
        for topic in CONTROLS:self.controls[topic]=self.create_publisher(empty,topic,10)

    def received(self,id,raw):
        stamp=self.get_clock().now().nanoseconds
        self.loop.call_soon_threadsafe(self.distribute,id,stamp,bytes(raw))

    def distribute(self,id,stamp,raw):
        self.latest[id]=(stamp,raw)
        for client in self.peers.values():
            for sub,channel in client['subs'].items():
                if channel==id:self.enqueue(client,bytes([1])+struct.pack('<IQ',sub,stamp)+raw)

    @staticmethod
    def enqueue(client,message):
        q=client['queue']
        if q.full():q.get_nowait()
        q.put_nowait(message)

    async def connection(self,ws):
        client=dict(subs={},publish={},queue=asyncio.Queue(maxsize=40));self.peers[ws]=client
        async def sender():
            while True:await ws.send(await client['queue'].get())
        task=None
        try:
            await ws.send(json.dumps(dict(op='serverInfo',name='yzbot Humble local visualization',capabilities=['clientPublish'],supportedEncodings=['cdr'],sessionId='yzbot-native')))
            await ws.send(json.dumps(dict(op='advertise',channels=list(self.channels.values()))))
            task=asyncio.create_task(sender())
            async for message in ws:
                if isinstance(message,str):
                    data=json.loads(message);operation=data.get('op')
                    if operation=='subscribe':
                        for s in data['subscriptions']:
                            sub,channel=int(s['id']),int(s['channelId'])
                            if channel not in self.channels:continue
                            client['subs'][sub]=channel
                            if channel in self.latest:
                                stamp,raw=self.latest[channel]
                                self.enqueue(client,bytes([1])+struct.pack('<IQ',sub,stamp)+raw)
                    elif operation=='unsubscribe':
                        for sub in data['subscriptionIds']:client['subs'].pop(sub,None)
                    elif operation=='advertise':
                        for c in data['channels']:
                            if c['topic'] not in CONTROLS or c.get('schemaName') not in ('std_msgs/msg/Empty','std_msgs/Empty'):
                                raise ValueError('只允许发布开始考试/停止的Empty消息')
                            client['publish'][c['id']]=c['topic']
                    elif operation=='unadvertise':
                        for id in data['channelIds']:client['publish'].pop(id,None)
                elif len(message)>=5 and message[0]==1:
                    id=struct.unpack_from('<I',message,1)[0]
                    topic=client['publish'].get(id)
                    if topic:self.controls[topic].publish(self.empty_type())
        except websockets.ConnectionClosed:pass
        except Exception as error:
            self.get_logger().error(str(error))
        finally:
            self.peers.pop(ws,None)
            if task:task.cancel()


async def main():
    rclpy.init();node=Bridge(asyncio.get_running_loop())
    thread=threading.Thread(target=lambda:rclpy.spin(node),daemon=True);thread.start()
    async with websockets.serve(node.connection,'127.0.0.1',8766,subprotocols=['foxglove.websocket.v1'],compression=None,max_size=4*1024*1024):
        print('Legacy Foxglove WebSocket v1 ready on ws://127.0.0.1:8766',flush=True)
        await asyncio.Future()


if __name__=='__main__':asyncio.run(main())
