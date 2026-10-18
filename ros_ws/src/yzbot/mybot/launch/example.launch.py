#!/usr/bin/env python
# -*- coding: utf-8 -*-

# _*_coding:utf-8_*_
# !/usr/bin/env python
import sys
import cv2
import time
import rospy
import os
import actionlib
from math import pow, sqrt, pi
from random import sample
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped, Twist
from std_srvs.srv import Empty
import subprocess
import tf
from std_msgs.msg import Bool


def photo1(area):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    ret, frame = cap.read()
    if not ret:
        print("无法读取摄像头帧")
        return
    image_path = os.path.join(os.path.dirname(__file__),
                              'image/photo/{area}_image.png'.format(area=area))
    height, width = frame.shape[:2]
    new_height = int(height * 0.23)
    cropped_frame = frame[new_height:, :]
    cv2.imwrite(image_path, cropped_frame)
    cap.release()
    cv2.destroyAllWindows()

class MultiGoalPublisher():
    def __init__(self):
        rospy.init_node('multi_goal_publisher', anonymous=True)

        # 设置清除代价地图服务
        self.clear_costmaps_service = None
        self.setup_costmap_service()
        
        # 优化move_base参数，提高导航稳定性
        self.optimize_move_base_params()

        # Set up move_base action client
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("等待move_base服务器...")
        wait_success = self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("已连接到move_base服务器")

        # 添加用于蒙特卡洛全局定位的发布器
        self.global_localization_pub = rospy.Publisher(
            '/global_localization', Bool, queue_size=1)
        
        # 添加初始化位姿估计的发布器
        self.initialpose_pub = rospy.Publisher(
            '/initialpose', PoseWithCovarianceStamped, queue_size=1)

        # 添加导航失败时的恢复机制
        self.recovery_behaviors = [
            self.clear_costmaps,  # 清除代价地图
            # self.rotate_recovery
        ]

        # Define the list of goal locations
        if (int(sys.argv[1]) == 1):
            self.locations = [
            
            Pose(Point(1.30,0.22,0),Quaternion(0.000, 0.000,0,1.0)),#黄线1 [0]

            Pose(Point(2.02,0.06,0),Quaternion(0.000, 0.000, 0,1.0)),#黄线2 [1]

            Pose(Point(3.03,0.14,0),Quaternion(0.000, 0.000, 0.72,0.70)),#turn1 [2]

            Pose(Point(2.98,0.73,0), Quaternion(0.000, 0.000, 1.0,0)),#turn2 [3]

            Pose(Point(2.96,0.77,0), Quaternion(0.000, 0.000, 1.0,0)),#turn2_5 [4]

            Pose(Point(2.50,0.70,0), Quaternion(0.000, 0.000, 0.70, 0.71)),#hole1 [5]
            
            Pose(Point(2.50,0.64,0), Quaternion(0.000, 0.000, 1.0, 0.0)),#hole_turn [6]
           
            Pose(Point(1.04,0.70,0), Quaternion(0.000, 0.000, 0.70, 0.72)),#hole2 [7]
            
            Pose(Point(1.00,0.58,0), Quaternion(0.000, 0.000, 1.0, -0.02)),#hole_turn2 [8]

            Pose(Point(0.03,0.7,0), Quaternion(0.000, 0.000, 0.71, 0.70)),#turn_corner [9]

            Pose(Point(0.19,1.163,0),Quaternion(0.000, 0.000, 0.705,0.709)),#黄线3 [10]

            Pose(Point(0.0,1.394,0),Quaternion(0.000, 0.000, 0.71,0.71)),#黄线4 [11]

            Pose(Point(0.23, 2.05, 0), Quaternion(0.000, 0.000, 0.20, 0.98)) , #start2 [12]

            Pose(Point(1.05, 2.15, 0), Quaternion(0.000, 0.000, 0.71, 0.71)),  #S1 [13]

            Pose(Point(1.01, 2.55, 0.00), Quaternion(0.000, 0.000, 0.70, 0.71)),  #S2 [14]

            Pose(Point(1.09, 2.91, 0.00), Quaternion(0.000, 0.000, 0.56, 0.83)),  #S3 [15]

            Pose(Point(1.25, 3.13, 0.00), Quaternion(0.000, 0.000, 0.42, 0.91)),  #S4 [16]

            Pose(Point(1.57, 3.33, 0.00), Quaternion(0.000, 0.000, -0.03, 1.0)),  #S5 [17]

            Pose(Point(2.09, 3.19, 0.00), Quaternion(0.000, 0.000, -0.45, 0.90)),  #S6 [18]

            Pose(Point(2.17, 3.0, 0.00), Quaternion(0.000, 0.000, -0.59, 0.81)),  #S7 [19]

            Pose(Point(2.20, 2.76, 0.00), Quaternion(0.000, 0.000, -0.68, 0.73)),  #S8 [20]

            Pose(Point(2.23, 2.27, 0.00), Quaternion(0.000, 0.000, -0.03, 1.0)),  #S9_turning [21]

            Pose(Point(3.01, 2.38, 0.00), Quaternion(0.000, 0.000, 0, 1.0)),  #start3 [22]

            Pose(Point(3.01, 2.38, 0.00), Quaternion(0.000, 0.000, 0.71, 0.70)),  #start3_2 [23]

            Pose(Point(3.01, 4.57, 0), Quaternion(0.000, 0.000, 1, 0)), #turn_north [24]
            
            Pose(Point(2.0,4.38,0), Quaternion(0.000, 0.000, -0.67, 0.74)),  #hole3 [25]
            
            Pose(Point(2.0,4.38,0), Quaternion(0.000, 0.000,1, 0)), #turn_west [26]
            
            Pose(Point(1.10,4.39,0), Quaternion(0.000, 0.000, -0.72, 0.70)), #hole4 [27]
            
            Pose(Point(1.05,4.31,0), Quaternion(0.000, 0.000, 1, 0)),#hole4_turn [28]
            
            Pose(Point(-0.30,4.38,0), Quaternion(0.000, 0.000, -0.71, 0.70)), #start4 [29]
            
            Pose(Point(-0.53, 1.97, 0), Quaternion(0.000, 0.000, -0.71, 0.71)), #middle [30]
            
            Pose(Point(-0.500,0.018,0), Quaternion(0.000, 0.000, -0.668, 0.744))#final [31]

                # Add more goal locations as needed
            ]

        elif (int(sys.argv[1]) == 2):
            self.locations = [
                Pose(Point(0.11, 2.02, 0), Quaternion(0.000, 0.000, 0.7, 0.7)),  #P0: start2

                Pose(Point(0.97, 2.19, 0), Quaternion(0.000, 0.000, 0.7, 0.7)),  #P1: S1

                Pose(Point(2.2, 2.23, 0), Quaternion(0.000, 0.000, -0.6, 0.8)),  #P2: S8

                Pose(Point(2.95, 2.3, 0), Quaternion(0.000, 0.000, 0.7, 0.7)),  #P3: start3

                Pose(Point(2.91, 4.57, 0), Quaternion(0.000, 0.000, 1, 0)),  #P4: turn_north

                Pose(Point(2.0, 4.36, 0), Quaternion(0.000, 0.000, 1, 0)),  #P5: hole3

                Pose(Point(1.05, 4.29, 0), Quaternion(0.000, 0.000, 1, 0)),  #P6: hole4

                Pose(Point(-0.3, 4.3, 0), Quaternion(0.000, 0.000, -0.7, 0.7)),  #P7: start4

                Pose(Point(-0.2, 3, 0), Quaternion(0.000, 0.000, -0.7, 0.7)),  #P8: middle

                Pose(Point(-0.35, 0.3, 0), Quaternion(0.000, 0.000, -0.67, 0.73))  #P9: final

                # Add more goal locations as needed
            ]

        elif (int(sys.argv[1]) == 3):
            self.locations = [

                Pose(Point(2.95, 2.3, 0), Quaternion(0.000, 0.000, 0.7, 0.7)),  #P0: start3

                Pose(Point(2.91, 4.57, 0), Quaternion(0.000, 0.000, 1, 0)),  #P1: turn_north

                Pose(Point(2.0, 4.36, 0), Quaternion(0.000, 0.000, 1, 0)),  #P2: hole3

                Pose(Point(1.05, 4.29, 0), Quaternion(0.000, 0.000, 1, 0)),  #P3: hole4

                Pose(Point(-0.3, 4.3, 0), Quaternion(0.000, 0.000, -0.7, 0.7)),  #P4: start4

                Pose(Point(-0.2, 3, 0), Quaternion(0.000, 0.000, -0.7, 0.7)),  #P5: middle

                Pose(Point(-0.35, 0.3, 0), Quaternion(0.000, 0.000, -0.67, 0.73))  #P6: final

                # Add more goal locations as needed
            ]
        elif (int(sys.argv[1]) == 4):
            self.locations = [
                Pose(Point(-0.3, 4.3, 0), Quaternion(0.000, 0.000, -0.7, 0.7)),  #P0: start4

                Pose(Point(-0.2, 3, 0), Quaternion(0.000, 0.000, -0.7, 0.7)),  #P1: middle

                Pose(Point(-0.35, 0.3, 0), Quaternion(0.000, 0.000, -0.67, 0.73))  #P2: final

                # Add more goal locations as needed
            ]

        # 删除全局重定位
        
        # Publish the goals
        self.publish_goals()

    def setup_costmap_service(self):
        """设置清除代价地图服务"""
        # 清除代价地图服务
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=5.0)
            self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            rospy.loginfo("清除代价地图服务已连接")
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logwarn("无法连接清除代价地图服务: %s", str(e))
            self.clear_costmaps_service = None
        
    def optimize_move_base_params(self):
        """优化move_base参数以提高导航稳定性"""
        rospy.loginfo("优化move_base参数...")
        
        try:
            # 增加规划器和控制器的耐心参数
            rospy.set_param("/move_base/planner_patience", 10.0)  # 默认5.0
            rospy.set_param("/move_base/controller_patience", 15.0)  # 默认5.0
            
            # 增加恢复行为的超时时间
            rospy.set_param("/move_base/recovery_behavior_enabled", True)
            rospy.set_param("/move_base/oscillation_timeout", 10.0)  # 默认5.0
            
            # 优化局部规划器参数
            rospy.set_param("/move_base/DWAPlannerROS/xy_goal_tolerance", 0.15)  # 默认0.1
            rospy.set_param("/move_base/DWAPlannerROS/yaw_goal_tolerance", 0.17)  # 默认0.1
            
            # 优化全局规划器参数
            rospy.set_param("/move_base/global_costmap/update_frequency", 2.0)  # 默认5.0
            rospy.set_param("/move_base/local_costmap/update_frequency", 5.0)  # 默认5.0
            
            # 减少障碍物膨胀半径，避免过度保守
            rospy.set_param("/move_base/global_costmap/inflation_radius", 0.30)  # 原来0.45，默认0.55
            rospy.set_param("/move_base/local_costmap/inflation_radius", 0.25)  # 原来0.4，默认0.55
            
            # 增加以下参数以调整避障敏感度
            # 增加障碍物代价缩放因子，使代价衰减更快
            rospy.set_param("/move_base/global_costmap/inflation_layer/cost_scaling_factor", 20.0)  # 默认10.0，值越大衰减越快
            rospy.set_param("/move_base/local_costmap/inflation_layer/cost_scaling_factor", 25.0)
            
            # 调整DWA规划器中的障碍物权重，减小对障碍物的敏感度
            rospy.set_param("/move_base/DWAPlannerROS/path_distance_bias", 32.0)  # 默认32.0
            rospy.set_param("/move_base/DWAPlannerROS/goal_distance_bias", 24.0)  # 默认24.0
            rospy.set_param("/move_base/DWAPlannerROS/occdist_scale", 0.01)  # 默认0.02，减小障碍物距离的权重
            
            # 稍微减小避障的反应距离
            rospy.set_param("/move_base/local_costmap/obstacle_layer/raytrace_range", 2.0)  # 默认3.0
            
            rospy.loginfo("move_base参数优化完成")
        except Exception as e:
            rospy.logwarn("无法设置move_base参数: %s" % str(e))

    # 删除重定位相关方法
        
    def clear_costmaps(self):
        """清除代价地图"""
        if self.clear_costmaps_service:
            try:
                rospy.loginfo("清除代价地图...")
                self.clear_costmaps_service()
                return True
            except rospy.ServiceException as e:
                rospy.logerr("清除代价地图调用失败: %s", str(e))
                return False
        return False
        
    def rotate_recovery(self):
        """恢复行为：原地旋转以获取更多的激光雷达数据 - 现在被禁用"""
        rospy.loginfo("旋转恢复行为已禁用")
        return True  # 直接返回成功，不执行旋转

    def try_recovery_behaviors(self):
        """尝试执行恢复行为"""
        # 修改恢复行为的顺序和执行方式
        rospy.loginfo("执行恢复行为序列...")
        
        # 1. 首先清除代价地图
        self.clear_costmaps()
        rospy.sleep(0.5)
        
        # 2. 尝试小幅度后退
        self.back_up_slightly()
        rospy.sleep(0.5)
        
        # 3. 尝试小幅度旋转
        self.rotate_slightly()
        
        # 注意：这里不再调用全局重定位，只在特定点上调用
        
        return True  # 总是返回True以继续导航
        
    def back_up_slightly(self):
        """小幅度后退以摆脱可能的障碍物"""
        rospy.loginfo("尝试小幅度后退...")
        
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.sleep(0.5)  # 等待发布器建立
        
        # 创建后退命令
        twist = Twist()
        twist.linear.x = -0.1  # 缓慢后退
        
        # 后退1秒
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        
        while (rospy.Time.now() - start_time).to_sec() < 1.0:
            cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # 停止
        twist.linear.x = 0.0
        cmd_vel_pub.publish(twist)
        
        return True
        
    def rotate_slightly(self):
        """小幅度旋转以获取更多传感器数据"""
        rospy.loginfo("尝试小幅度旋转...")
        
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.sleep(0.5)  # 等待发布器建立
        
        # 创建旋转命令
        twist = Twist()
        twist.angular.z = 0.5  # 缓慢旋转
        
        # 旋转1秒
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        
        while (rospy.Time.now() - start_time).to_sec() < 1.0:
            cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # 停止
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
        
        # 反方向旋转回来
        twist.angular.z = -0.5
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 1.0:
            cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # 停止
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
        
        return True

    # 删除全面重定位方法

    def rotate_360_degrees(self):
        """让小车原地顺时针旋转一圈"""
        rospy.loginfo("开始执行顺时针360度旋转...")
        
        # 创建一个发布Twist消息的发布器
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 等待发布器建立
        rospy.sleep(1.0)
        
        # 创建Twist消息
        twist = Twist()
        twist.linear.x = 0.0  # 不进行线性移动
        twist.angular.z = -1.0  # 负值表示顺时针旋转，速度提高
        
        # 计算旋转时间
        rotation_time = 2 * pi / abs(twist.angular.z)  # 旋转一圈所需的时间
        
        # 发布旋转命令
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        
        while (rospy.Time.now() - start_time).to_sec() < rotation_time:
            cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # 停止旋转
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
        
        rospy.loginfo("顺时针360度旋转完成")
        
        # 给系统一些时间稳定
        rospy.sleep(1.0)
        
        return True

    def publish_goals(self):
        num = 0
        # 删除重定位点设置
        last_pose = None
        
        # 更新需要旋转的点
        rotation_points = [3,22,29]  # turn2
        
        # 更新需要额外等待的点
        wait_points = [2, 9, 19, 29]  # 复杂转弯点
        
        # 更新需要减速的点
        slow_points = [2, 6, 9, 19, 29]  # 复杂区域减速
        
        # 添加需要进行蒙特卡洛重定位的点
        montecarlo_points = [0, 10, 20, 27]  # 需要重定位的点

        if (int(sys.argv[1]) == 1):
            num = 0
        elif (int(sys.argv[1]) == 2):
            num = 7
        elif (int(sys.argv[1]) == 3):
            num = 11
        elif (int(sys.argv[1]) == 4):
            num = 16

        for i, goal_pose in enumerate(self.locations):
            # 在每个目标前清除代价地图
            self.clear_costmaps()
            
            # 检查当前点是否需要进行蒙特卡洛重定位
            if i in montecarlo_points:
                rospy.loginfo("在点 %d 执行蒙特卡洛全局重定位...", num)
                self.perform_monte_carlo_localization()
                rospy.sleep(2.0)  # 给重定位更多时间稳定
            
            # Create a goal message
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = goal_pose

            # 发送目标前等待一会儿，让定位更加稳定
            rospy.sleep(0.5)
            
            # 在复杂区域多等待一会
            if i in wait_points:
                rospy.loginfo("导航点 %d 是复杂区域，多等待一会..." % num)
                rospy.sleep(1.0)
            
            # 照片拍摄代码保持不变
            # if num == 5:
            #     photo1('A')
            #     print("拍摄成功")
            #     subprocess.Popen(['conda', 'run', '-n', 'ican', 'python',
            #                      '/home/mowen/newznzc_ws/src/nav_demo/scripts/PGX/identify_soldier0.py', 'A','best1.onnx'])
            #     print("检测成功")

            # if num == 7:
            #     photo1('B')
            #     print("拍摄成功")
            #     subprocess.Popen(['conda', 'run', '-n', 'ican', 'python',
            #                      '/home/mowen/newznzc_ws/src/nav_demo/scripts/PGX/identify_soldier0.py', 'B','best1.onnx'])
            #     print("检测成功")
                
            # if num == 21:
            #     photo1('C')
            #     print("拍摄成功")
            #     subprocess.Popen(['conda', 'run', '-n', 'ican', 'python',
            #                      '/home/mowen/newznzc_ws/src/nav_demo/scripts/PGX/identify_soldier0.py', 'C','best1.onnx'])
            #     print("检测成功")
            
            # if num == 23:
            #     photo1('D')
            #     print("拍摄成功")
            #     subprocess.Popen(['conda', 'run', '-n', 'ican', 'python',
            #                      '/home/mowen/newznzc_ws/src/nav_demo/scripts/PGX/identify_soldier0.py', 'D','best1.onnx'])

            # 在复杂区域减速
            if i in slow_points:
                rospy.loginfo("导航点 %d 是复杂区域，减速导航..." % num)
                # 设置更保守的导航参数
                try:
                    rospy.set_param("/move_base/DWAPlannerROS/max_vel_x", 0.3)  # 降低最大速度
                    rospy.set_param("/move_base/DWAPlannerROS/max_vel_theta", 0.5)  # 降低最大角速度
                except Exception as e:
                    rospy.logwarn("无法设置减速参数: %s" % str(e))
            else:
                # 恢复正常速度
                try:
                    rospy.set_param("/move_base/DWAPlannerROS/max_vel_x", 0.5)  # 恢复最大速度
                    rospy.set_param("/move_base/DWAPlannerROS/max_vel_theta", 1.0)  # 恢复最大角速度
                except Exception as e:
                    rospy.logwarn("无法恢复速度参数: %s" % str(e))
            
            # 发送目标并处理导航失败情况
            success = self.send_goal_with_recovery(goal)
            
            # 保存最后一个成功的位姿
            if success:
                last_pose = goal_pose
                
                # 如果当前点是需要旋转的点，执行旋转
                if i in rotation_points:
                    rospy.loginfo("到达旋转点 %d，开始执行360度旋转..." % num)
                    self.rotate_360_degrees()
                    # 旋转后再次清除代价地图
                    self.clear_costmaps()
            
            # 删除特定点重定位代码
            if not success:
                # 如果导航失败，尝试恢复
                rospy.loginfo("导航点 %d 失败，尝试恢复..." % num)
                self.clear_costmaps()
                
                # 如果导航失败，尝试从上一个成功的点重新规划
                if last_pose is not None:
                    rospy.loginfo("尝试从上一个成功点重新规划...")
                    # 创建一个中间点，距离上一个点稍近一些
                    intermediate_goal = MoveBaseGoal()
                    intermediate_goal.target_pose.header.frame_id = 'map'
                    intermediate_goal.target_pose.header.stamp = rospy.Time.now()
                    intermediate_goal.target_pose.pose = last_pose
                    
                    # 尝试导航到中间点
                    success = self.send_goal_with_recovery(intermediate_goal)
                    if success:
                        rospy.loginfo("成功回到上一个点，重新尝试当前目标...")
                        # 重新尝试当前目标
                        success = self.send_goal_with_recovery(goal)
            
            num += 1
            i += 1
            
    def navigate_to_pose(self, pose, is_transition=False):
        """导航到指定位姿，过渡点使用更宽松的参数"""
        # 创建目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        
        # 如果是过渡点，使用更宽松的参数
        if is_transition:
            try:
                # 临时保存当前参数
                orig_xy_tolerance = rospy.get_param("/move_base/DWAPlannerROS/xy_goal_tolerance")
                orig_yaw_tolerance = rospy.get_param("/move_base/DWAPlannerROS/yaw_goal_tolerance")
                
                # 设置更宽松的参数
                rospy.set_param("/move_base/DWAPlannerROS/xy_goal_tolerance", 0.25)  # 更宽松的位置容差
                rospy.set_param("/move_base/DWAPlannerROS/yaw_goal_tolerance", 0.35)  # 更宽松的角度容差
            except Exception as e:
                rospy.logwarn("无法设置过渡点参数: %s" % str(e))
        
        # 发送目标
        success = self.send_goal_with_recovery(goal)
        
        # 如果是过渡点，恢复原来的参数
        if is_transition:
            try:
                rospy.set_param("/move_base/DWAPlannerROS/xy_goal_tolerance", orig_xy_tolerance)
                rospy.set_param("/move_base/DWAPlannerROS/yaw_goal_tolerance", orig_yaw_tolerance)
            except Exception as e:
                rospy.logwarn("无法恢复参数: %s" % str(e))
                
        return success

    def send_goal_with_recovery(self, goal):
        """发送目标并在失败时执行恢复行为"""
        # 发送目标
        self.move_base.send_goal(goal)
        
        # 增加超时时间
        max_wait_time = 120.0  # 秒，增加到120秒
        wait_increment = 1.0
        total_wait_time = 0.0
        
        # 添加失败计数器，避免无限循环
        failure_count = 0
        max_failures = 3  # 增加到3次
        
        # 添加周期性清除代价地图的计时器
        last_costmap_clear_time = rospy.Time.now()
        costmap_clear_interval = 20.0  # 每20秒清除一次代价地图
        
        while total_wait_time < max_wait_time and failure_count < max_failures:
            state = self.move_base.get_state()
            
            # 周期性清除代价地图
            if (rospy.Time.now() - last_costmap_clear_time).to_sec() > costmap_clear_interval:
                rospy.loginfo("周期性清除代价地图...")
                self.clear_costmaps()
                last_costmap_clear_time = rospy.Time.now()
            
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("到达目标点！")
                return True
                
            if state == actionlib.GoalStatus.ABORTED:
                failure_count += 1
                rospy.logwarn("导航被中止 (失败 %d/%d)，尝试恢复..." % (failure_count, max_failures))
                self.move_base.cancel_goal()
                
                # 执行基本恢复行为（不包括全局重定位）
                if self.try_recovery_behaviors():
                    # 恢复成功，重新发送目标
                    rospy.sleep(1.0)  # 给系统一些时间稳定
                    self.move_base.send_goal(goal)
            
            # 检查是否卡在原地
            elif state == actionlib.GoalStatus.ACTIVE and total_wait_time > 30.0:
                # 每30秒检查一次是否取得进展
                rospy.loginfo("导航进行中，已用时 %.1f 秒..." % total_wait_time)
                
            rospy.sleep(wait_increment)
            total_wait_time += wait_increment
            
        # 如果超时或失败次数过多，取消当前目标
        self.move_base.cancel_goal()
        
        if total_wait_time >= max_wait_time:
            rospy.logwarn("导航超时 (%.1f 秒)，取消目标" % total_wait_time)
        elif failure_count >= max_failures:
            rospy.logwarn("导航失败次数过多 (%d)，放弃此目标" % failure_count)
            
        return False

    def perform_monte_carlo_localization(self):
        """执行蒙特卡洛全局重定位"""
        rospy.loginfo("执行蒙特卡洛全局重定位...")
        
        # 发送全局定位请求
        global_loc_msg = Bool()
        global_loc_msg.data = True
        
        # 发布多次确保消息被接收
        for _ in range(3):
            self.global_localization_pub.publish(global_loc_msg)
            rospy.sleep(0.1)
        
        # 给AMCL足够的时间进行重定位
        rospy.loginfo("等待蒙特卡洛重定位完成...")
        rospy.sleep(2.0)  # 增加等待时间
        
        # 移除360度旋转
        # self.rotate_360_degrees()
        
        # 清除代价地图
        self.clear_costmaps()
        
        rospy.loginfo("蒙特卡洛全局重定位完成")
        return True
        
    def set_initial_pose(self, pose):
        """设置机器人的初始位姿估计"""
        rospy.loginfo("设置初始位姿...")
        
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.pose.pose = pose
        
        # 设置协方差矩阵 (不确定性)
        # 对角线上值较小表示更确定
        for i in range(36):
            if i == 0 or i == 7 or i == 14:  # x, y, z 位置
                initial_pose.pose.covariance[i] = 0.1
            elif i == 21 or i == 28 or i == 35:  # 旋转
                initial_pose.pose.covariance[i] = 0.2
            else:
                initial_pose.pose.covariance[i] = 0.0
        
        # 发布初始位姿
        self.initialpose_pub.publish(initial_pose)
        rospy.sleep(1.0)  # 给系统时间处理
        
        rospy.loginfo("初始位姿已设置")
        return True


if __name__ == '__main__':
    try:
        MultiGoalPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Multi-goal navigation finished.")

