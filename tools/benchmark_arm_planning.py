#!/usr/bin/env python3
"""Read-only MoveIt planning comparison; never sends a robot trajectory."""
import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints,JointConstraint
rclpy.init();node=Node('arm_planning_benchmark');latest=[]
node.create_subscription(JointState,'/joint_states',lambda msg:latest.append(msg),10)
client=node.create_client(GetMotionPlan,'/plan_kinematic_path')
if not client.wait_for_service(timeout_sec=5):raise RuntimeError('MoveIt unavailable')
deadline=time.monotonic()+5
while not latest and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
if not latest:raise RuntimeError('No real joint state')
state=latest[-1];names=[f'joint{i}' for i in range(1,7)];values=dict(zip(state.name,state.position))
target=[values[name] for name in names];target[1]+=.15
records=[]
for attempts in (5,1,5,1,5,1):
 req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='arm';r.start_state.joint_state=state
 r.allowed_planning_time=3.;r.num_planning_attempts=attempts
 r.max_velocity_scaling_factor=.9;r.max_acceleration_scaling_factor=.9
 r.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=name,position=float(value),tolerance_above=.002,tolerance_below=.002,weight=1.) for name,value in zip(names,target)])]
 start=time.monotonic();future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=10)
 if not future.done():raise RuntimeError('Planning timeout')
 result=future.result().motion_plan_response;points=result.trajectory.joint_trajectory.points
 duration=points[-1].time_from_start if points else None
 records.append({'attempts':attempts,'wall_seconds':time.monotonic()-start,'reported_planning_seconds':result.planning_time,'result_code':result.error_code.val,'trajectory_points':len(points),'trajectory_seconds':duration.sec+duration.nanosec/1e9 if duration else None})
report={'purpose':'read-only plans from same real joint state; no trajectory execution','initial_joint_values':values,'target':target,'records':records}
out=Path('/opt/yzbot/verification/arm-planning-comparison-20261020.json');out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
node.destroy_node();rclpy.shutdown()
