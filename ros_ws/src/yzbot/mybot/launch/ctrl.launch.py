#!/usr/bin/python3
import os
import subprocess
import json
import rclpy
import time
import threading
from rclpy.node import Node
from std_msgs.msg import String, Int16
from geometry_msgs.msg import PoseStamped, Point
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from math import sqrt
from launch import LaunchDescription
from launch.actions import (
    TimerAction,
    ExecuteProcess
)
from gazebo_msgs.srv import GetEntityState


class CubeCoordinateManager(Node):
    def __init__(self):
        super().__init__('cube_coordinate_manager')
        self.cube_coordinates = {
            'red': [],
            'blue': []
        }
        self.target_zones = {}
        self.robot_current_pose = None
        self.current_goal = None
        self.nav = None
        self.navigation_active = False
        self.processed_cubes = {  # 跟踪已处理的方块
            'red': [],
            'blue': []
        }

        # 服务客户端
        self.client = self.create_client(GetEntityState, '/demo/get_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务 /demo/get_entity_state 不可用，等待中...')

        # 订阅指令
        self.command_sub = self.create_subscription(
            String,
            '/llama_command',
            self.command_callback,
            10
        )
        
        # 状态发布者
        self.nav_status_pub = self.create_publisher(Int16, '/nav_status', 10)
        self.arm_status_pub = self.create_publisher(Int16, '/arm_status', 10)
        
        # 初始化导航
        self.initialize_navigator()
        
        # 分步初始化变量
        self.temp_coords = None
        self.get_logger().info("开始初始化坐标获取...")
        self.init_step = 0
        self.init_timer = self.create_timer(1.0, self.step_init)

    def step_init(self):
        if self.init_step == 0:
            self.get_logger().info("步骤1/3：获取小车初始位置")
            self.get_model_coordinates_async("six_arm", callback=self.set_robot_initial)
            self.init_step += 1
        elif self.init_step == 1:
            self.get_logger().info("步骤2/3：获取目标点位置")
            self.targets_to_get = ['zone_a', 'zone_b', 'zone_c']
            self.get_next_target()
            self.init_step += 1
        elif self.init_step == 2:
            self.get_logger().info("步骤3/3：获取所有方块坐标")
            self.cubes_to_get = [('red', i) for i in range(1,6)] + [('blue', i) for i in range(1,6)]
            self.get_next_cube()
            self.init_step += 1
        elif self.init_step == 3:
            self.get_logger().info("所有坐标初始化完成")
            self.init_timer.destroy()
            # 初始化时不启动小车位置更新定时器，避免重复输出坐标

    def set_robot_initial(self, coords):
        if coords:
            self.robot_current_pose = coords
            self.get_logger().info(f"小车初始位置: x={coords['x']:.4f}, y={coords['y']:.4f}, z={coords['z']:.4f}")
        else:
            self.get_logger().warn("无法获取小车初始位置，使用默认值(0,0,0)")
            self.robot_current_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def get_next_target(self):
        if not self.targets_to_get:
            self.get_logger().info(f"目标点位置获取完成: {self.target_zones}")
            return
        zone = self.targets_to_get.pop(0)
        self.get_model_coordinates_async(zone, callback=lambda coords: self.save_target(zone, coords))

    def save_target(self, zone, coords):
        if coords:
            self.target_zones[zone] = coords
        self.get_next_target()

    def get_next_cube(self):
        if not self.cubes_to_get:
            self.get_logger().info("所有方块坐标获取完成")
            self.get_logger().info(f"红色方块数量: {len(self.cube_coordinates['red'])}")
            self.get_logger().info(f"蓝色方块数量: {len(self.cube_coordinates['blue'])}")
            return
        color, idx = self.cubes_to_get.pop(0)
        self.get_model_coordinates_async(
            f"{color}_cube_{idx}",
            callback=lambda coords: self.save_cube(color, idx, coords)
        )

    def save_cube(self, color, idx, coords):
        if coords:
            self.cube_coordinates[color].append({
                'index': idx,
                'x': coords['x'],
                'y': coords['y'],
                'z': coords['z']
            })
        # 用临时定时器替代once参数
        timer = self.create_timer(0.2, self.get_next_cube)
        def destroy_timer():
            timer.destroy()
        self.create_timer(0.21, destroy_timer)

    def get_model_coordinates_async(self, model_name, callback):
        try:
            request = GetEntityState.Request()
            request.name = model_name
            request.reference_frame = 'world'

            future = self.client.call_async(request)
            future.add_done_callback(lambda f: self.handle_coords_response(f, model_name, callback))
        except Exception as e:
            self.get_logger().error(f"异步获取{model_name}坐标失败: {str(e)}")
            callback(None)

    def handle_coords_response(self, future, model_name, callback):
        try:
            if future.done():
                response = future.result()
                if response.success:
                    pos = response.state.pose.position
                    coords = {
                        'x': pos.x,
                        'y': pos.y,
                        'z': pos.z
                    }
                    # 仅在初始化阶段输出坐标，避免后续重复输出
                    if self.init_step < 3:
                        self.get_logger().info(f"=== {model_name} 坐标 ===")
                        self.get_logger().info(f"x: {coords['x']:.4f}, y: {coords['y']:.4f}, z: {coords['z']:.4f}")
                    callback(coords)
                    return
                else:
                    self.get_logger().warning(f'获取 {model_name} 失败：实体不存在')
            else:
                self.get_logger().error(f'获取 {model_name} 坐标超时')
        except Exception as e:
            self.get_logger().error(f"处理{model_name}响应失败: {str(e)}")
        callback(None)

    def update_robot_position_async(self):
        self.get_model_coordinates_async("six_arm", callback=self.update_robot_pose)

    def update_robot_pose(self, new_pose):
        if new_pose:
            self.robot_current_pose = new_pose

    def initialize_navigator(self):
        if not rclpy.ok():
            rclpy.init(args=None)
        self.nav = BasicNavigator()
        self.get_logger().info("等待导航系统启动...")
        self.nav.waitUntilNav2Active()
        self.get_logger().info("导航系统已激活")

    def calculate_distance(self, x1, y1, x2, y2):
        return sqrt((x1 - x2)**2 + (y1 - y2)** 2)

    def find_optimal_cube(self, color, target_coords):
        """计算所有剩余物块的距离明细，并返回最优方块"""
        if color not in self.cube_coordinates:
            self.get_logger().error(f"无效的颜色: {color}")
            return None
            
        if len(self.cube_coordinates[color]) == 0:
            self.get_logger().error(f"没有找到{color}方块")
            return None
        
        # 强制更新小车当前位置，确保数据最新
        self.update_robot_position_async()
        rclpy.spin_once(self, timeout_sec=0.2)  # 等待位置更新完成
        if not self.robot_current_pose:
            self.get_logger().warn("无法获取小车位置，使用初始位置计算距离")
            return None
        
        # 打印小车当前位置
        self.get_logger().info(f"\n===== 小车当前位置 =====")
        self.get_logger().info(f"x: {self.robot_current_pose['x']:.4f}, y: {self.robot_current_pose['y']:.4f}")
        self.get_logger().info(f"目标区域位置: x: {target_coords['x']:.4f}, y: {target_coords['y']:.4f}")
        
        # 筛选未处理的方块
        available_cubes = [
            cube for cube in self.cube_coordinates[color]
            if cube['index'] not in self.processed_cubes[color]
        ]
        
        if not available_cubes:
            self.get_logger().info(f"所有{color}方块已处理完毕")
            return None
        
        # 打印所有剩余物块的距离明细
        self.get_logger().info(f"\n===== 剩余{color}方块距离明细 =====")
        cube_costs = []
        for cube in available_cubes:
            # 计算三段距离
            to_cube = self.calculate_distance(
                self.robot_current_pose['x'], self.robot_current_pose['y'],
                cube['x'], cube['y']
            )
            cube_to_target = self.calculate_distance(
                cube['x'], cube['y'],
                target_coords['x'], target_coords['y']
            )
            total_cost = to_cube + cube_to_target
            
            # 打印当前方块的距离信息
            self.get_logger().info(
                f"方块{color}_cube_{cube['index']}: "
                f"小车到方块距离={to_cube:.4f}m, "
                f"方块到目标距离={cube_to_target:.4f}m, "
                f"总距离={total_cost:.4f}m"
            )
            cube_costs.append((cube, total_cost))
        
        # 按总成本排序，选择最小的
        cube_costs.sort(key=lambda x: x[1])
        best_cube, best_cost = cube_costs[0]
        self.get_logger().info(f"\n===== 选中最优方块 =====")
        self.get_logger().info(f"方块{color}_cube_{best_cube['index']}，总距离{best_cost:.4f}m")
        
        return best_cube

    def navigate_to_coordinate(self, x, y, goal_type):
        if not self.nav:
            self.initialize_navigator()
            
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0  # 朝向正方向
        
        self.get_logger().info(f"\n===== 发布导航目标点 =====")
        self.get_logger().info(f"目标坐标: x={x:.4f}, y={y:.4f}")
        self.current_goal = {'x': x, 'y': y, 'type': goal_type}
        self.navigation_active = True
        
        time.sleep(0.5)
        self.nav.goToPose(goal_pose)
        
        # 等待导航任务启动（不输出重复信息）
        start_time = time.time()
        while not self.nav.isTaskComplete() and time.time() - start_time < 5.0:
            rclpy.spin_once(self, timeout_sec=0.5)
        
        # 导航循环
        while not self.nav.isTaskComplete() and self.navigation_active:
            self.update_robot_position_async()
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # 超时处理
            feedback = self.nav.getFeedback()
            if feedback and Duration.from_msg(feedback.navigation_time) > Duration(seconds=600):
                self.get_logger().warn("导航超时，取消任务")
                self.nav.cancelTask()
                self.navigation_active = False
                return False
            
            # 检查是否到达目标附近（不输出实时距离）
            if self.robot_current_pose:
                distance = self.calculate_distance(
                    self.robot_current_pose['x'], 
                    self.robot_current_pose['y'],
                    self.current_goal['x'],
                    self.current_goal['y']
                )
                
                # 到达目标附近
                if distance < 0.8:
                    self.get_logger().info(f"到达目标附近 (距离: {distance:.2f}m)，停止导航")
                    self.nav.cancelTask()
                    self.navigation_active = False
                    
                    # 发布导航状态
                    if 'zone_' in self.current_goal.get('type', ''):
                        self.nav_status_pub.publish(Int16(data=2))  # 到达仓库
                    else:
                        self.nav_status_pub.publish(Int16(data=1))  # 到达目标点
                    break
            
            time.sleep(0.1)
        
        # 检查导航结果
        result = self.nav.getResult()
        self.navigation_active = False
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("导航成功")
            if 'zone_' in self.current_goal.get('type', ''):
                self.nav_status_pub.publish(Int16(data=2))
            else:
                self.nav_status_pub.publish(Int16(data=1))
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("导航被取消")
        elif result == TaskResult.FAILED:
            self.get_logger().error("导航失败")
        else:
            self.get_logger().error("导航结果无效")
            
        return False

    def run_command_with_timeout(self, cmd, timeout):
        result = {'success': False, 'output': None, 'error': None}
        
        def target():
            try:
                proc = subprocess.Popen(
                    cmd, 
                    shell=True, 
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE, 
                    text=True
                )
                stdout, stderr = proc.communicate()
                result['success'] = proc.returncode == 0
                result['output'] = stdout
                result['error'] = stderr
            except Exception as e:
                result['error'] = str(e)
        
        thread = threading.Thread(target=target)
        thread.start()
        thread.join(timeout)
        
        if thread.is_alive():
            self.get_logger().warn(f"命令执行超时（{timeout}秒），尝试终止进程")
            return False
        
        if not result['success']:
            self.get_logger().error(f"命令执行失败: {result['error']}")
        
        return result['success']

    def execute_arm_command(self, command_type):
        max_retries = 3
        timeout = 7.0
        
        if command_type == 'lower':
            cmd = """ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],points: [ { positions: [0.0, 1.35, 0.8, 0.0, -0.99, 0.0], time_from_start: { sec: 1, nanosec: 500 } },] }}" """ 
        elif command_type == 'raise':
            cmd = """ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{ trajectory: { joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [ { positions: [0.0, 0.63, 0.96, 0.0, -1.53, 0.0], time_from_start: { sec: 1, nanosec: 500 } },] }}" """
        else:
            self.get_logger().error(f"无效的机械臂指令类型: {command_type}")
            return False
        
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f"\n===== 执行机械臂{command_type}动作 ===== (尝试 {attempt + 1}/{max_retries})")
                success = self.run_command_with_timeout(cmd, timeout)
                
                if success:
                    self.get_logger().info(f"机械臂{command_type}动作执行完成")
                    return True
                else:
                    self.get_logger().warn(f"机械臂{command_type}动作执行未成功，{'将重试' if attempt < max_retries - 1 else '已达到最大重试次数'}")
            
            except Exception as e:
                self.get_logger().error(f"执行机械臂动作失败: {str(e)}, {'将重试' if attempt < max_retries - 1 else '已达到最大重试次数'}")
            
            if attempt < max_retries - 1:
                self.get_logger().info("等待1秒后重试...")
        
        return False

    def execute_gripper_command(self, command_type):
        max_retries = 3
        timeout = 10.0
        
        if command_type == 'pick':
            cmd = """ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [finger_joint1],
    points: [
      { positions: [0.2], time_from_start: { sec: 1, nanosec: 200 } }
    ]
  }
}" """
        elif command_type == 'release':
            cmd = """ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [finger_joint1],
    points: [
      { positions: [0.0], time_from_start: { sec: 1, nanosec: 200 } }
    ]
  }
}" """
        else:
            self.get_logger().error(f"无效的夹爪指令类型: {command_type}")
            return False
        
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f"\n===== 执行夹爪{command_type}动作 ===== (尝试 {attempt + 1}/{max_retries})")
                success = self.run_command_with_timeout(cmd, timeout)
                
                if success:
                    self.get_logger().info(f"夹爪{command_type}动作执行完成")
                    return True
                else:
                    self.get_logger().warn(f"夹爪{command_type}动作执行未成功，{'将重试' if attempt < max_retries - 1 else '已达到最大重试次数'}")
            
            except Exception as e:
                self.get_logger().error(f"执行夹爪动作失败: {str(e)}, {'将重试' if attempt < max_retries - 1 else '已达到最大重试次数'}")
            
            if attempt < max_retries - 1:
                self.get_logger().info("等待1秒后重试...")
        
        return False

    def attach_cube(self, color, index):
        max_retries = 3
        timeout = 7.0
        
        cube_model = f"{color}_cube_{index}"
        
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f"\n===== 连接物体: {cube_model} ===== (尝试 {attempt + 1}/{max_retries})")
                cmd1 = f"""ros2 service call /ATTACHLINK linkattacher_msgs/srv/AttachLink "{{model1_name: 'six_arm', link1_name: 'link6', model2_name: '{cube_model}', link2_name: 'link'}}" """
                success = self.run_command_with_timeout(cmd1, timeout)
                
                if success:
                    self.get_logger().info(f"物体{cube_model}连接完成")
                    return True
                else:
                    self.get_logger().warn(f"物体{cube_model}连接未成功，{'将重试' if attempt < max_retries - 1 else '已达到最大重试次数'}")
            
            except Exception as e:
                self.get_logger().error(f"连接物体失败: {str(e)}, {'将重试' if attempt < max_retries - 1 else '已达到最大重试次数'}")
            
            if attempt < max_retries - 1:
                self.get_logger().info("等待1秒后重试...")
        
        return False

    def detach_cube(self, color, index):
        max_retries = 3
        timeout = 7.0
        
        cube_model = f"{color}_cube_{index}"
        
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f"\n===== 取消连接物体: {cube_model} ===== (尝试 {attempt + 1}/{max_retries})")
                cmd1 = f"""ros2 service call /DETACHLINK linkattacher_msgs/srv/DetachLink "{{model1_name: 'six_arm', link1_name: 'link6', model2_name: '{cube_model}', link2_name: 'link'}}" """
                success = self.run_command_with_timeout(cmd1, timeout)
                
                if success:
                    self.get_logger().info(f"物体{cube_model}取消连接完成")
                    return True
                else:
                    self.get_logger().warn(f"物体{cube_model}取消连接未成功，{'将重试' if attempt < max_retries - 1 else '已达到最大重试次数'}")
            
            except Exception as e:
                self.get_logger().error(f"取消连接物体失败: {str(e)}, {'将重试' if attempt < max_retries - 1 else '已达到最大重试次数'}")
            
            if attempt < max_retries - 1:
                self.get_logger().info("等待1秒后重试...")
        
        return False

    def process_cube_pickup(self, color, index):
        self.get_logger().info(f"\n===== 开始拾取{color}方块 {index} =====")
        
        if not self.execute_arm_command('lower'):
            return False
        
        if not self.execute_gripper_command('pick'):
            return False
        
        if not self.attach_cube(color, index):
            return False
        
        if not self.execute_arm_command('raise'):
            return False
            
        self.get_logger().info(f"{color}方块 {index} 拾取完成")
        self.arm_status_pub.publish(Int16(data=1))
        return True

    def process_cube_placement(self, color, index):
        self.get_logger().info(f"\n===== 开始放置{color}方块 {index} =====")
        
        if not self.execute_arm_command('lower'):
            return False
        
        if not self.execute_gripper_command('release'):
            return False
        
        if not self.detach_cube(color, index):
            return False
        
        if not self.execute_arm_command('raise'):
            return False
            
        self.get_logger().info(f"{color}方块 {index} 放置完成")
        self.arm_status_pub.publish(Int16(data=2))
        return True

    def command_callback(self, msg):
        try:
            self.get_logger().info(f"\n\n===== 收到新指令 =====")
            self.get_logger().info(f"原始指令: {msg.data}")
            commands = json.loads(msg.data)
            
            for cmd in commands:
                color = cmd.get('color', '').lower()
                num = cmd.get('num', 1)
                target_area = cmd.get('to', '').lower()
                
                self.get_logger().info(f"解析指令: 移动 {num} 个 {color} 方块到 {target_area} 区")
                
                target_zone = f"zone_{target_area}"
                if target_zone not in self.target_zones:
                    self.get_logger().error(f"目标区域{target_area}不存在（可用区域：a/b/c）")
                    continue
                    
                target_coords = self.target_zones[target_zone]
                self.get_logger().info(f"目标区域{target_area}坐标: x={target_coords['x']:.4f}, y={target_coords['y']:.4f}")
                
                # 重置该颜色已处理的方块列表（针对新任务）
                self.processed_cubes[color] = []
                
                # 处理指定数量的方块
                for i in range(num):
                    self.get_logger().info(f"\n===== 处理第{i+1}/{num}个{color}方块 =====")
                    
                    # 每次处理前重新计算最优方块（基于当前小车位置）
                    best_cube = self.find_optimal_cube(color, target_coords)
                    if not best_cube:
                        self.get_logger().error(f"没有找到可用的{color}方块，终止当前任务")
                        break
                    
                    cube_index = best_cube['index']
                    self.get_logger().info(f"选中方块: {color}_cube_{cube_index}")
                    
                    # 导航到方块位置（偏移0.4）
                    target_x = best_cube['x'] - 0.3
                    target_y = best_cube['y']
                    self.get_logger().info(f"导航目标偏移坐标: x={target_x:.4f}, y={target_y:.4f}")
                    success = self.navigate_to_coordinate(target_x, target_y, "cube")
                    if not success:
                        self.get_logger().error(f"导航到方块失败，跳过该方块")
                        continue
                    time.sleep(1.0)
                    
                    # 拾取方块
                    if not self.process_cube_pickup(color, cube_index):
                        self.get_logger().error(f"方块拾取失败，跳过该方块")
                        continue
                    
                    # 导航到目标区域
                    self.get_logger().info(f"导航到目标区域{target_area}")
                    success = self.navigate_to_coordinate(target_coords['x'], target_coords['y'], target_zone)
                    if not success:
                        self.get_logger().error(f"导航到目标区域失败，跳过该方块放置流程")
                        continue
                    time.sleep(1.0)
                    
                    # 放置方块
                    if not self.process_cube_placement(color, cube_index):
                        self.get_logger().error(f"方块放置失败")
                        continue
                    
                    # 标记为已处理
                    self.processed_cubes[color].append(cube_index)
                    self.get_logger().info(f"已完成{len(self.processed_cubes[color])}/{num}个{color}方块处理")
                    time.sleep(1.0)
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON解析错误: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"处理指令时出错: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = CubeCoordinateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，退出程序")
    finally:
        node.destroy_node()
        rclpy.shutdown()

def generate_launch_description():
    current_file_path = os.path.abspath(__file__)
    
    ld = LaunchDescription()
    
    ld.add_action(TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', current_file_path],
                output='screen'
            )
        ]
    ))
    
    return ld

if __name__ == '__main__':
    main()
