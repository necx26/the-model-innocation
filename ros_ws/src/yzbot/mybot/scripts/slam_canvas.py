#!/usr/bin/env python3
"""Fuse the supplied map prior with online SLAM in the SLAM coordinate frame.

The fixed canvas permits goals in not-yet-observed rooms. Live SLAM overwrites
observed prior cells; patrol costs are added only to free cells. Original scene
geometry and map files are never changed by this node.
"""
import copy
import math
from pathlib import Path
import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
from scene_config import PATROLS


class Canvas(Node):
    def __init__(self):
        super().__init__('slam_map_canvas')
        self.tf=Buffer(node=self);self.listener=TransformListener(self.tf,self)
        maps=Path(get_package_share_directory('bot_navigation'))/'maps'
        self.meta=yaml.safe_load((maps/'map.yaml').read_text())
        self.prior=cv2.imread(str(maps/self.meta['image']),0)
        self.res=.05;self.xmin=-8.;self.ymin=-7.;self.width=320;self.height=280
        self.xs=self.xmin+np.arange(self.width)*self.res
        self.ys=self.ymin+np.arange(self.height)*self.res
        self.xx,self.yy=np.meshgrid(self.xs,self.ys)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub=self.create_publisher(OccupancyGrid,'/map',qos)
        self.nav_pub=self.create_publisher(OccupancyGrid,'/competition/navigation_map',qos)
        self.create_subscription(OccupancyGrid,'/slam/map',self.update_map,qos)

    def update_map(self,source):
        try:t=self.tf.lookup_transform('map','odom',Time()).transform
        except Exception:return
        q=t.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        c,s=math.cos(yaw),math.sin(yaw)
        dx=self.xx-t.translation.x;dy=self.yy-t.translation.y
        ox=c*dx+s*dy;oy=-s*dx+c*dy
        columns=np.rint((ox-self.meta['origin'][0])/self.meta['resolution']).astype(int)
        rows=self.prior.shape[0]-1-np.rint((oy-self.meta['origin'][1])/self.meta['resolution']).astype(int)
        good=(rows>=0)&(rows<self.prior.shape[0])&(columns>=0)&(columns<self.prior.shape[1])
        prior=np.full((self.height,self.width),-1,dtype=np.int8)
        values=self.prior[np.clip(rows,0,self.prior.shape[0]-1),np.clip(columns,0,self.prior.shape[1]-1)]
        prior[good&(values>250)]=0;prior[good&(values<100)]=100
        # Raster the latest SLAM occupancy grid onto the fixed map frame canvas.
        if abs(source.info.resolution-self.res)>.0001:return
        sr=np.asarray(source.data,dtype=np.int8).reshape(source.info.height,source.info.width)
        x=int(round((source.info.origin.position.x-self.xmin)/self.res));y=int(round((source.info.origin.position.y-self.ymin)/self.res))
        x0=max(0,x);y0=max(0,y);x1=min(self.width,x+sr.shape[1]);y1=min(self.height,y+sr.shape[0])
        raw=np.full_like(prior,-1)
        if x0<x1 and y0<y1:raw[y0:y1,x0:x1]=sr[y0-y:y1-y,x0-x:x1-x]
        navigation=np.where(raw>=0,raw,prior).astype(np.int8)
        for p in PATROLS:
            a,b=p['start'],p['end']
            risk=(ox>min(a[0],b[0])-.65)&(ox<max(a[0],b[0])+.65)&(oy>min(a[1],b[1])-.65)&(oy<max(a[1],b[1])+.65)
            navigation[risk&(navigation>=0)&(navigation<60)]=60
        msg=OccupancyGrid();msg.header=copy.deepcopy(source.header)
        msg.info=copy.deepcopy(source.info);msg.info.width=self.width;msg.info.height=self.height
        msg.info.origin.position.x=self.xmin;msg.info.origin.position.y=self.ymin
        msg.info.origin.orientation.x=msg.info.origin.orientation.y=msg.info.origin.orientation.z=0.;msg.info.origin.orientation.w=1.
        msg.data=raw.ravel().tolist();self.map_pub.publish(msg)
        msg.data=navigation.ravel().tolist();self.nav_pub.publish(msg)


if __name__=='__main__':
    rclpy.init();node=Canvas()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
