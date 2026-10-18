import rclpy,time,json,math
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
rclpy.init();n=Node('simulation_inspector');d={};b=Buffer();t=TransformListener(b,n)
def model(m):
 for i,name in enumerate(m.name):
  if name=='six_arm':
   p=m.pose[i];q=p.orientation
   d['world']=[p.position.x,p.position.y,p.position.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))];d['vel']=str(m.twist[i])
n.create_subscription(ModelStates,'/demo/model_states_demo',model,1)
n.create_subscription(Twist,'/cmd_vel',lambda m:d.update(cmd=[m.linear.x,m.angular.z]),1)
n.create_subscription(Odometry,'/odom',lambda m:d.update(odom=[m.pose.pose.position.x,m.pose.pose.position.y]),1)
end=time.monotonic()+3
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
for frame in ['map','odom']:
 try:
  p=b.lookup_transform(frame,'base_control',Time()).transform;d[frame+'_tf']=str(p)
 except Exception as e:d[frame+'_tf']=str(e)
print(json.dumps(d,indent=2));n.destroy_node();rclpy.shutdown()
