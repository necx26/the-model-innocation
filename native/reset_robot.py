import time
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Twist

rclpy.init();n=Node('pre_slam_spawn_reset');c=n.create_client(SetEntityState,'/demo/set_entity_state')
if not c.wait_for_service(timeout_sec=10):raise SystemExit('Gazebo实体复位服务不可用')
s=EntityState(name='six_arm',reference_frame='world');s.pose.orientation.w=1.
f=c.call_async(SetEntityState.Request(state=s));rclpy.spin_until_future_complete(n,f,timeout_sec=8)
if not f.done() or not f.result().success:raise SystemExit('SLAM启动前复位失败')
p=n.create_publisher(Twist,'/cmd_vel',10)
for _ in range(20):p.publish(Twist());rclpy.spin_once(n,timeout_sec=.05)
n.destroy_node();rclpy.shutdown()
