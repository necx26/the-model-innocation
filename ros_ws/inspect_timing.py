import rclpy,time,json
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
rclpy.init();n=Node('timing_inspector');data={};count={}
def add(name,t):
 data[name]=t.sec+t.nanosec/1e9;count[name]=count.get(name,0)+1
n.create_subscription(Clock,'/clock',lambda m:add('clock',m.clock),qos_profile_sensor_data)
n.create_subscription(LaserScan,'/scan',lambda m:add('scan',m.header.stamp),qos_profile_sensor_data)
def tf(m):
 for t in m.transforms:
  if (t.header.frame_id,t.child_frame_id) in [('map','odom'),('odom','base_footprint')]:add(t.header.frame_id+'_'+t.child_frame_id,t.header.stamp)
n.create_subscription(TFMessage,'/tf',tf,100)
end=time.monotonic()+5
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
print(json.dumps({'last':data,'counts_5sec':count},indent=2));n.destroy_node();rclpy.shutdown()
