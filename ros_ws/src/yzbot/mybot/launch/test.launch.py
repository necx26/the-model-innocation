import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
import xacro
import re

def generate_launch_description():
    """生成launch描述"""
    return LaunchDescription([
        # 启动第一个turtlesim节点，命名空间为turtlesim1
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        
        # 启动第二个turtlesim节点，命名空间为turtlesim2
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        
        # 启动mimic节点，用于让第二个turtlesim模仿第一个turtlesim的动作
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            output='screen',  # 在屏幕上显示输出
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),  # 输入位姿话题重映射
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel')  # 输出速度指令话题重映射
            ]
        )
    ])
