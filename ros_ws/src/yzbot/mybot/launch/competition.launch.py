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
    mode=os.environ.get('YZBOT_LOCALIZATION','truth')
    if mode=='slam':
        params_file=os.path.join(get_package_share_directory('slam_toolbox'),'config','mapper_params_online_async.yaml')
        localization=[
            Node(package='slam_toolbox', executable='async_slam_toolbox_node',name='slam_toolbox',output='screen',
                 parameters=[params_file,{'use_sim_time':True,'base_frame':'base_control','odom_frame':'odom','map_frame':'map',
                    'scan_topic':'/scan','use_map_saver':False,'enable_interactive_mode':False,'map_update_interval':.5,
                    'minimum_time_interval':.1,'minimum_travel_distance':.2,'minimum_travel_heading':.15}],
                 remappings=[('/map','/slam/map'),('/map_metadata','/slam/map_metadata')]),
            Node(package='mybot',executable='slam_canvas.py',parameters=[{'use_sim_time':True}],output='screen')]
    else:
        localization=[
            Node(package='nav2_map_server',executable='map_server',name='patrol_map_server',
                 parameters=[{'use_sim_time':True,'topic_name':'/competition/navigation_map','frame_id':'map',
                 'yaml_filename':os.path.join(nav,'maps','competition_safe_map.yaml')}]),
            Node(package='nav2_map_server',executable='map_server',name='practice_map_server',
                 parameters=[{'use_sim_time':True,'topic_name':'/map','frame_id':'map',
                 'yaml_filename':os.path.join(nav,'maps','map.yaml')}]),
            Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='patrol_map_lifecycle',
                 parameters=[{'use_sim_time':True,'autostart':True,'node_names':['patrol_map_server','practice_map_server']}]),
            Node(package='tf2_ros',executable='static_transform_publisher',name='simulation_world_alignment',
                 arguments=['0','0','0','0','0','0','map','odom'])]
    return LaunchDescription(localization+[
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
