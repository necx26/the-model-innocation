import math,time,json,rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK,GetStateValidity
from builtin_interfaces.msg import Duration
rclpy.init();n=Node('ik_inspector');j={}
n.create_subscription(JointState,'/joint_states',lambda m:j.update(zip(m.name,m.position)),30)
end=time.monotonic()+2
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
p=[.273,.04,.12];r=math.hypot(p[0],p[1]);z=p[2]+.06-.2834;e=math.acos((r*r+z*z-.14**2-.28**2)/(2*.14*.28));a=math.atan2(r,z)-math.atan2(.28*math.sin(e),.14+.28*math.cos(e));phi=math.atan2(p[1],p[0]);v=[phi,a,e,0,a+e-math.pi,phi]
for k,x in enumerate(v,1):j['joint'+str(k)]=x
req=GetPositionIK.Request();q=req.ik_request;q.group_name='arm';q.ik_link_name='grasping_frame';q.robot_state.joint_state=JointState(name=list(j),position=list(j.values()));q.pose_stamped.header.frame_id='base_footprint';q.pose_stamped.pose.position.x,q.pose_stamped.pose.position.y,q.pose_stamped.pose.position.z=p;q.pose_stamped.pose.orientation.y=1.;q.pose_stamped.pose.orientation.w=0.;q.timeout=Duration(sec=2);q.avoid_collisions=False
c=n.create_client(GetPositionIK,'/compute_ik');assert c.wait_for_service(timeout_sec=5);f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5);ik=f.result();print('IK without collision:',ik.error_code.val)
state=ik.solution if ik.error_code.val==1 else q.robot_state
valid=n.create_client(GetStateValidity,'/check_state_validity');assert valid.wait_for_service(timeout_sec=5);f=valid.call_async(GetStateValidity.Request(robot_state=state,group_name='arm'));rclpy.spin_until_future_complete(n,f,timeout_sec=5);a=f.result();print('Validity:',a.valid);print('Contacts:',[(c.contact_body_1,c.contact_body_2) for c in a.contacts]);print('Joints',list(zip(state.joint_state.name,state.joint_state.position)))
n.destroy_node();rclpy.shutdown()
