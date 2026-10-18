#!/usr/bin/python3
# 组合启动文件：同时启动Gazebo、Nav2导航、MoveIt!机械臂规划控制和红蓝色块识别
import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# 红蓝色块识别节点
class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        # 创建图像订阅者
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.listener_callback, 10)
        # 创建处理后图像的发布者
        self.pub = self.create_publisher(Image, '/detected_image', 10)
        self.cv_bridge = CvBridge()
        self.get_logger().info("红蓝色块识别节点已启动")

    def listener_callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV格式
            image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
            # 进行色块检测和标注
            processed_image = self.object_detect(image)
            # 将处理后的图像转换为ROS消息并发布
            self.pub.publish(self.cv_bridge.cv2_to_imgmsg(processed_image, 'bgr8'))
        except Exception as e:
            self.get_logger().error(f"处理图像时出错: {str(e)}")

    def object_detect(self, image):
        # 降低画质：缩小图像尺寸（保持视角范围不变，仅降低分辨率）
        low_quality_size = (320, 180)  # 原1280×720的1/4尺寸（宽高各减半）
        scaled_image = cv2.resize(
            image, 
            low_quality_size, 
            interpolation=cv2.INTER_NEAREST  # 最近邻插值，进一步降低画质
        )
        
        # 定义红色的HSV阈值范围
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        
        # 定义蓝色的HSV阈值范围
        lower_blue = np.array([97, 181, 0])
        upper_blue = np.array([179, 255, 255])
        
        # 转换为HSV颜色空间（基于缩放后的低画质图像）
        hsv_img = cv2.cvtColor(scaled_image, cv2.COLOR_BGR2HSV)
        
        # 创建红色的掩码
        mask_red1 = cv2.inRange(hsv_img, lower_red, upper_red)
        mask_red2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        # 创建蓝色的掩码
        mask_blue = cv2.inRange(hsv_img, lower_blue, upper_blue)
        
        # 红色轮廓检测与标注
        contours_red, _ = cv2.findContours(
            mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_red:
            if cv2.contourArea(cnt) < 50:  # 过滤小轮廓（适配低分辨率调整阈值）
                continue
            (x, y, w, h) = cv2.boundingRect(cnt)
            cv2.drawContours(scaled_image, [cnt], -1, (0, 255, 0), 2)  # 红色方块用绿色轮廓
            center_x, center_y = int(x + w / 2), int(y + h / 2)
            area = cv2.contourArea(cnt)  # 计算轮廓面积
            cv2.circle(scaled_image, (center_x, center_y), 3, (0, 0, 255), -1)  # 红色中心点
            cv2.putText(scaled_image, f"R:({center_x}, {center_y}) Area:{int(area)}", 
                       (center_x + 5, center_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)
        
        # 蓝色轮廓检测与标注
        contours_blue, _ = cv2.findContours(
            mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_blue:
            if cv2.contourArea(cnt) < 50:  # 过滤小轮廓（适配低分辨率调整阈值）
                continue
            (x, y, w, h) = cv2.boundingRect(cnt)
            cv2.drawContours(scaled_image, [cnt], -1, (0, 0, 255), 2)  # 蓝色方块用红色轮廓
            center_x, center_y = int(x + w / 2), int(y + h / 2)
            area = cv2.contourArea(cnt)  # 计算轮廓面积
            cv2.circle(scaled_image, (center_x, center_y), 3, (0, 255, 0), -1)  # 蓝色中心点
            cv2.putText(scaled_image, f"B:({center_x}, {center_y}) Area:{int(area)}", 
                       (center_x + 5, center_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
        
        return scaled_image

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber('color_detector')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def generate_launch_description():
    # 获取包的路径
    mybot_dir = get_package_share_directory('mybot')
    bot_navigation_dir = get_package_share_directory('bot_navigation')
    # 获取当前文件路径
    current_file_path = os.path.abspath(__file__)
 
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 1. 启动Gazebo仿真环境（立即启动）
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mybot_dir, 'launch', 'gazebo_world.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    ))
    
    # 2. 延迟启动Nav2导航系统
    ld.add_action(TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bot_navigation_dir, 'launch', 'nav_bringup_gazebo.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    ))
    
    # 3. 延迟启动MoveIt!机械臂规划控制
    ld.add_action(TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(mybot_dir, 'launch', 'my_moveit_rviz.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    ))
    
    # 4. 延迟启动色块识别节点
    ld.add_action(TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', current_file_path],
                output='screen'
            )
        ]
    ))
    
    # 添加启动参数声明
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间'
    ))
    
    return ld

if __name__ == '__main__':
    main()
