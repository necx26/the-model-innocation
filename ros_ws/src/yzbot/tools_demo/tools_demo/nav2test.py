from geometry_msgs.msg import PoseStamped

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

rclpy.init()
nav = BasicNavigator()
# 等待导航启动完成
nav.waitUntilNav2Active()
# 设置目标点坐标
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.header.stamp = nav.get_clock().now().to_msg()
goal_pose.pose.position.x = -1.0
goal_pose.pose.position.y = -1.0
goal_pose.pose.orientation.w = -1.0
# 发送目标接收反馈结果
nav.goToPose(goal_pose)
while not nav.isTaskComplete():
    feedback = nav.getFeedback()
    nav.get_logger().info('导航...')
# 超时自动取消
if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600):  # type: ignore
    nav.cancelTask()
    # 最终结果判断
result = nav.getResult()
if result == TaskResult.SUCCEEDED:
    nav.get_logger().info('导航结果: 成功')
elif result == TaskResult.CANCELED:
    nav.get_logger().warn('导航结果: 被取消')
elif result == TaskResult.FAILED:
    nav.get_logger().error('导航结果: 失败')
else:
    nav.get_logger().error('导航结果: 返回状态无效')
