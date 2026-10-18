import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = (MoveItConfigsBuilder('six_arm', package_name='mybot')
              .robot_description(file_path='config/six_arm_full.urdf.xacro')
              .robot_description_semantic(file_path='config/six_arm_full.srdf')
              .to_moveit_configs())
    config.planning_pipelines['ompl'].setdefault('arm', {})['longest_valid_segment_fraction'] = 0.005
    nav = get_package_share_directory('bot_navigation')
    bringup = get_package_share_directory('nav2_bringup')
    return LaunchDescription([
        Node(package='nav2_map_server', executable='map_server', name='patrol_map_server',
             parameters=[{'use_sim_time': True, 'topic_name': '/competition/safe_map',
                          'yaml_filename': os.path.join(nav, 'maps', 'competition_safe_map.yaml'),
                          'frame_id': 'map'}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='patrol_map_lifecycle', parameters=[{'use_sim_time': True,
                 'autostart': True, 'node_names': ['patrol_map_server', 'practice_map_server']}]),
        Node(package='nav2_map_server', executable='map_server', name='practice_map_server',
             parameters=[{'use_sim_time': True, 'topic_name': 'map',
                          'yaml_filename': os.path.join(nav, 'maps', 'map.yaml'),
                          'frame_id': 'map'}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='simulation_world_alignment',
             arguments=['0','0','0','0','0','0','map','odom']),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[config.to_dict(), {'use_sim_time': True,
                         'publish_robot_description_semantic': True}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'navigation_launch.py')),
            launch_arguments={'map': os.path.join(nav, 'maps', 'map.yaml'),
                              'params_file': os.path.join(nav, 'param', 'competition_nav2.yaml'),
                              'use_sim_time': 'true', 'use_composition': 'False',
                              'autostart': 'true'}.items()),
    ])
