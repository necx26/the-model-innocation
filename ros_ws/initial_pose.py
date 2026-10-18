#!/usr/bin/env python3
"""Give AMCL the robot's known Gazebo spawn pose in the supplied map."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.parameter import Parameter
from lifecycle_msgs.srv import GetState


def main():
    rclpy.init()
    node = rclpy.create_node(
        "yzbot_initial_pose",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
        automatically_declare_parameters_from_overrides=True,
    )
    publisher = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    deadline = time.monotonic() + 45
    lifecycle = node.create_client(GetState, '/amcl/get_state')
    active = False
    while time.monotonic() < deadline:
        if not lifecycle.wait_for_service(timeout_sec=.5):
            continue
        future = lifecycle.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.)
        if future.done() and future.result() and future.result().current_state.id == 3:
            active = True
            break
        time.sleep(.2)
    if not active:
        raise RuntimeError('AMCL did not become active')
    while rclpy.ok() and publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if publisher.get_subscription_count() == 0:
        raise RuntimeError("AMCL did not subscribe to /initialpose")
    while rclpy.ok() and node.get_clock().now().nanoseconds == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.get_clock().now().nanoseconds == 0:
        raise RuntimeError("Simulation clock did not start")

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.pose.pose.orientation.w = 1.0
    pose.pose.covariance[0] = 0.25
    pose.pose.covariance[7] = 0.25
    pose.pose.covariance[35] = (math.pi / 6) ** 2
    for _ in range(12):
        pose.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(pose)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(.2)
    print(f"Published initial pose at simulation time {pose.header.stamp.sec}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
