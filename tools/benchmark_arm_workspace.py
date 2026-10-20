#!/usr/bin/env python3
"""Read-only reach and collision-aware planning probe; never executes motion."""
import json,math,threading,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from tf2_ros import Buffer,TransformListener
from moveit_msgs.srv import GetPositionIK,GetMotionPlan
from competition_node import Competition
from robot_geometry import square_gripper_yaw
class Probe(Node):
    robot_pose=Competition.robot_pose
    robot_state=Competition.robot_state
    solve_ik=Competition.solve_ik
    plan_arm=Competition.plan_arm
    def __init__(self):
        super().__init__('arm_workspace_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.joints={};self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.create_subscription(JointState,'/joint_states',lambda m:self.joints.update(zip(m.name,m.position)),10)
        self.ik=self.create_client(GetPositionIK,'/compute_ik');self.planner=self.create_client(GetMotionPlan,'/plan_kinematic_path')
    def call(self,client,request,timeout=12):
        if not client.wait_for_service(timeout_sec=3):raise RuntimeError(client.srv_name)
        future=client.call_async(request);end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:time.sleep(.01)
        if not future.done():raise TimeoutError(client.srv_name)
        return future.result()
rclpy.init();node=Probe();ex=MultiThreadedExecutor(num_threads=2);ex.add_node(node)
thread=threading.Thread(target=ex.spin,daemon=True);thread.start()
try:
    end=time.monotonic()+10
    while (not node.robot_pose() or len(node.joints)<6) and time.monotonic()<end:time.sleep(.1)
    pose=node.robot_pose();records=[]
    for name,x,y in [('front',.24,0.),('left',0.,.31),('right',0.,-.31),('rear',-.42,0.)]:
        wx=pose[0]+x*math.cos(pose[2])-y*math.sin(pose[2]);wy=pose[1]+x*math.sin(pose[2])+y*math.cos(pose[2])
        record=dict(direction=name,local_target=[x,y],base_pose=pose)
        try:
            joints=[node.solve_ik(wx,wy,z,world_yaw=square_gripper_yaw(pose[2])) for z in (.18,.10)]
            state=node.robot_state();durations=[]
            for values,speed in zip(joints,(1.,.85)):
                traj=node.plan_arm(values,speed,state);last=traj.points[-1]
                data=dict(zip(traj.joint_names,last.positions));state.joint_state.position=[data.get(j,v) for j,v in zip(state.joint_state.name,state.joint_state.position)]
                durations.append(last.time_from_start.sec+last.time_from_start.nanosec/1e9)
            record.update(planned=True,joint_targets=joints,trajectory_seconds=durations)
        except Exception as error:record.update(planned=False,error=str(error))
        records.append(record)
    report=dict(purpose='read-only IK and collision-aware plans from actual current base; no motion execution',records=records)
    out=Path('/opt/yzbot/verification/arm-workspace.json');out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
finally:
    ex.shutdown();thread.join();node.destroy_node();rclpy.shutdown()
