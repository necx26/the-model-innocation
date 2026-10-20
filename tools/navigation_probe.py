#!/usr/bin/env python3
"""Record requested/actual base motion and optionally test one Nav2 goal."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--goal', type=float, nargs=3)
    parser.add_argument('--goal-frame',choices=['map','odom'],default='odom')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('competition_navigation_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    latest = {}
    subscriptions = []
    for topic in ('/cmd_vel', '/cmd_vel_nav'):
        def make_callback(topic):
            def callback(msg):
                latest[topic] = [round(msg.linear.x, 4), round(msg.angular.z, 4)]
                latest[topic+'_at'] = time.monotonic()
            return callback
        subscriptions.append(node.create_subscription(Twist, topic, make_callback(topic), 10))
    def odom(msg):
        latest['actual'] = [round(msg.twist.twist.linear.x, 4), round(msg.twist.twist.angular.z, 4)]
    subscriptions.append(node.create_subscription(Odometry, '/odom', odom, 10))
    def plan(msg):
        latest['plan'] = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses[::max(1,len(msg.poses)//40)]]
    subscriptions.append(node.create_subscription(NavPath, '/plan', plan, 10))
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    started = time.monotonic()
    result = None
    handle = None
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w') as stream:
        while time.monotonic()-started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.01)
            if args.goal and handle is None and time.monotonic()-started > 2:
                if client.wait_for_server(timeout_sec=1):
                    x,y,yaw=args.goal
                    goal=NavigateToPose.Goal()
                    goal.pose.header.frame_id=args.goal_frame
                    goal.pose.pose.position.x=x;goal.pose.pose.position.y=y
                    goal.pose.pose.orientation.z=math.sin(yaw/2)
                    goal.pose.pose.orientation.w=math.cos(yaw/2)
                    pending=client.send_goal_async(goal)
                    rclpy.spin_until_future_complete(node,pending,timeout_sec=5)
                    handle=pending.result()
                    result=handle.get_result_async() if handle and handle.accepted else None
                    if result is None:raise RuntimeError('goal rejected')
            now=time.monotonic()
            if now-latest.get('_sample',0) < .2:continue
            latest['_sample']=now
            sample={k:v for k,v in latest.items() if not k.endswith('_at') and k!='_sample'}
            sample['t']=round(now-started,3)
            try:
                t=buffer.lookup_transform('odom','base_control',Time()).transform
                q=t.rotation
                sample['pose']=[round(t.translation.x,4),round(t.translation.y,4),round(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),4)]
            except Exception:pass
            if result is not None and result.done():sample['result_status']=result.result().status
            stream.write(json.dumps(sample)+'\n');stream.flush()
            if 'result_status' in sample:break
        if handle is not None and result is not None and not result.done():
            pending=handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node,pending,timeout_sec=4)
    print(str(out),flush=True)
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
