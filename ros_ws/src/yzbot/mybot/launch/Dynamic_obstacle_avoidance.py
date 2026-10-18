import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import math
import time

# 显示模式控制变量
SHOW_MODE = 0

# ROS 2参数服务相关导入
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# 全局参数设置
# 实体尺寸参数
CAR_RADIUS = 0.4  # 小车半径(m) → 直径0.8m
OBSTACLE_RADIUS = 0.3  # 障碍物半径(m) → 直径0.5m
EXTENSION = 0.0  # 危险区域扩展距离(m)
SAFE_EDGE_DISTANCE = 0.3  # 边缘到边缘的安全距离(m)

# 检测参数
OBSTACLE_DISTANCE_THRESHOLD = 3.0  # 障碍物与交点的距离阈值(m)
CAR_OBSTACLE_DISTANCE_THRESHOLD = 4.0  # 小车与障碍物中心点距离阈值(m)
UPDATE_INTERVAL = 50  # 更新间隔(ms)，对应20Hz
RESUME_DISTANCE_THRESHOLD = CAR_OBSTACLE_DISTANCE_THRESHOLD * 1.2  # 滞后恢复阈值

# 障碍物运动边界
OBSTACLE1_X_MIN = -5.5  # 障碍物1左边界（X轴水平运动）
OBSTACLE1_X_MAX = -1.5  # 障碍物1右边界（X轴水平运动）
OBSTACLE2_Y_MIN = -1.0   # 障碍物2下边界（Y轴垂直运动）
OBSTACLE2_Y_MAX = 3.0   # 障碍物2上边界（Y轴垂直运动）

REVERSE_THRESHOLD = 0.5  # 接近边界的预判阈值(m)
SAFE_TIME_THRESHOLD = 3.0  # 安全时间差阈值(s)

# 预测参数
PREDICTION_TIME = 2.0  # 预测总时间(s)
PREDICTION_STEP = 0.2  # 预测步长(s)
PREDICTION_STEPS = int(PREDICTION_TIME / PREDICTION_STEP)  # 预测步数
MIN_SPEED_THRESHOLD = 0.01  # 最小速度阈值

# 可视化参数
TRAIL_LENGTH = 20  # 轨迹记录长度


class CollisionAvoidanceSystem(Node):
    def __init__(self, root):
        super().__init__('collision_avoidance_system')
        self.root = root
        self.root.title("Collision Avoidance with Path-based Prediction")

        # 实体状态存储
        self.path_points = []  # 导航轨迹点集
        self.path_distances = []  # 导航轨迹上各点的累积距离
        self.car_pose = None
        self.car_prev_pose = None
        self.car_speed = 0.0
        self.car_direction = (0.0, 0.0)
        self.car_yaw = 0.0  # 小车偏航角
        self.car_path_index = 0  # 小车在导航轨迹上的索引

        # 障碍物状态
        self.obs1_pos = None
        self.obs1_prev_pos = None
        self.obs1_speed = 0.0
        self.obs1_direction = (0.0, 0.0)
        self.obs2_pos = None
        self.obs2_prev_pos = None
        self.obs2_speed = 0.0
        self.obs2_direction = (0.0, 0.0)

        # 碰撞风险相关
        self.red_zones = []  # 危险区域
        self.obstacle_center_lines = []  # 障碍物移动轨迹中心线
        self.collision_point = None  # 预测碰撞点
        self.time_diff = None  # 时间差
        self.edge_distance = None  # 边缘到边缘的距离
        self.colliding_obstacle = None  # 发生碰撞风险的障碍物ID
        
        # 预测位置存储
        self.predicted_car_positions = []  # 预测的小车位置
        self.predicted_obs_positions = {1: [], 2: []}  # 预测的障碍物位置
        self.collision_prediction = None  # 预测的碰撞信息

        # 轨迹记录
        self.obs_trail = {1: [], 2: []}  # 障碍物运动轨迹
        self.car_trail = []  # 小车运动轨迹
        self.last_known_car_pos = (0, 0)  # 记录小车最后已知位置

        # 时间戳和计数器
        self.prev_update_time = time.time()
        self.log_counter = 0
        self.error_counter = 0  # 错误计数

        # ROS初始化
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.gazebo_client = self.create_client(GetEntityState, '/demo/get_entity_state')
        while not self.gazebo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /demo/get_entity_state 服务...')

        # 参数客户端配置
        self.initial_vel_params = {
            "max_vel_x": 1.1,
            "max_vel_theta": 0.7,
            "max_speed_xy": 1.0
        }
        self.stop_vel_params = {
            "max_vel_x": 0.0,
            "max_vel_theta": 0.01,
            "max_speed_xy": 0.0
        }
        self.param_client = self.create_client(SetParameters, "/controller_server/set_parameters")
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /controller_server/set_parameters 服务...')
        self.is_stopped = False  # 停止状态标记

        # 绘图初始化
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.path_line, = self.ax.plot([], [], 'r-', linewidth=2, label='Navigation Path')
        
        # 小车（圆形+轨迹+速度箭头）
        self.car_circle = plt.Circle((0, 0), CAR_RADIUS, fill=False, edgecolor='blue', linewidth=2, label='Car (D=0.8m)')
        self.car_dir_arrow = plt.Arrow(0, 0, 0, 0, width=0.15, color='blue')
        self.car_trail_line, = self.ax.plot([], [], 'b--', linewidth=1, alpha=0.5, label='Car Trail')
        
        # 障碍物1（圆形+轨迹+速度箭头）
        self.obs1_circle = plt.Circle((0, 0), OBSTACLE_RADIUS, fill=False, edgecolor='green', linewidth=2, label='Obstacle 1 (D=0.5m)')
        self.obs1_dir_arrow = plt.Arrow(0, 0, 0, 0, width=0.15, color='green')
        self.obs1_trail_line, = self.ax.plot([], [], 'g--', linewidth=1, alpha=0.5, label='Obstacle 1 Trail')
        
        # 障碍物2（圆形+轨迹+速度箭头）
        self.obs2_circle = plt.Circle((0, 0), OBSTACLE_RADIUS, fill=False, edgecolor='purple', linewidth=2, label='Obstacle 2 (D=0.5m)')
        self.obs2_dir_arrow = plt.Arrow(0, 0, 0, 0, width=0.15, color='purple')
        self.obs2_trail_line, = self.ax.plot([], [], 'm--', linewidth=1, alpha=0.5, label='Obstacle 2 Trail')
        
        # 预测位置可视化
        self.predicted_car_markers = [plt.Circle((0,0), CAR_RADIUS*0.8, fill=True, color='blue', alpha=0.3) 
                                     for _ in range(PREDICTION_STEPS)]
        self.predicted_obs1_markers = [plt.Circle((0,0), OBSTACLE_RADIUS*0.8, fill=True, color='green', alpha=0.3) 
                                     for _ in range(PREDICTION_STEPS)]
        self.predicted_obs2_markers = [plt.Circle((0,0), OBSTACLE_RADIUS*0.8, fill=True, color='purple', alpha=0.3) 
                                     for _ in range(PREDICTION_STEPS)]
                                     
        for marker in self.predicted_car_markers + self.predicted_obs1_markers + self.predicted_obs2_markers:
            self.ax.add_patch(marker)
        
        # 危险区域和障碍物轨迹线
        self.red_zone_patches = [plt.Rectangle((0,0),0,0,fill=True,color='red',alpha=0.3) for _ in range(4)]
        self.future_red_zone_patches = [plt.Rectangle((0,0),0,0,fill=True,color='pink',alpha=0.3) for _ in range(4)]
        self.obstacle1_center_line, = self.ax.plot([], [], 'k-', linewidth=1.5, label='Obstacle 1 Center Line')
        self.obstacle2_center_line, = self.ax.plot([], [], 'k-.', linewidth=1.5, label='Obstacle 2 Center Line')
        for patch in self.red_zone_patches + self.future_red_zone_patches:
            self.ax.add_patch(patch)
        
        # 碰撞风险标记
        self.collision_marker, = self.ax.plot([], [], 'yo', markersize=8, label='Predicted Collision Point')
        self.time_diff_text = self.ax.text(0, 0, '', fontsize=10, color='orange')
        self.edge_distance_text = self.ax.text(0, 0, '', fontsize=10, color='purple')
        
        # 安全距离预警线
        self.safe_distance_circle = plt.Circle((0, 0), 
                                              CAR_OBSTACLE_DISTANCE_THRESHOLD, 
                                              fill=False, edgecolor='orange', linestyle='--', linewidth=1,
                                              label='Safety Distance (Center)')

        # 添加绘图元素
        self.ax.add_patch(self.car_circle)
        self.ax.add_patch(self.car_dir_arrow)
        self.ax.add_patch(self.obs1_circle)
        self.ax.add_patch(self.obs1_dir_arrow)
        self.ax.add_patch(self.obs2_circle)
        self.ax.add_patch(self.obs2_dir_arrow)
        self.ax.add_patch(self.safe_distance_circle)

        # 坐标轴与图例
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Collision Avoidance (Path-based Prediction)')
        self.ax.grid(True, linestyle=':')
        self.ax.legend(loc='upper right', fontsize=8)
        self.ax.set_aspect('equal')

        # Tkinter画布
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=1)

        # 启动主循环
        self.root.after(UPDATE_INTERVAL, self.update_all)

    def path_callback(self, msg):
        """处理导航路径消息，计算路径点和累积距离"""
        self.path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        
        # 计算路径上各点的累积距离
        self.path_distances = [0.0]  # 第一个点距离为0
        total_dist = 0.0
        for i in range(1, len(self.path_points)):
            x1, y1 = self.path_points[i-1]
            x2, y2 = self.path_points[i]
            dist = math.hypot(x2 - x1, y2 - y1)
            total_dist += dist
            self.path_distances.append(total_dist)

    def find_closest_point_on_path(self, x, y):
        """找到导航路径上离指定点最近的点及其索引"""
        if not self.path_points:
            return 0, (0, 0)
            
        min_dist = float('inf')
        closest_idx = 0
        
        # 从当前索引附近开始搜索，提高效率
        start_idx = max(0, self.car_path_index - 5)
        end_idx = min(len(self.path_points), self.car_path_index + 20)
        
        for i in range(start_idx, end_idx):
            px, py = self.path_points[i]
            dist = math.hypot(px - x, py - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        return closest_idx, self.path_points[closest_idx]

    def get_position_along_path(self, start_idx, distance):
        """根据距离获取导航路径上的位置"""
        if not self.path_points or len(self.path_points) < 2:
            return None
            
        # 如果距离超过路径总长度，返回最后一个点
        if distance >= self.path_distances[-1]:
            return self.path_points[-1]
            
        # 找到距离对应的路径段
        idx = start_idx
        while idx < len(self.path_distances) - 1 and self.path_distances[idx] < distance:
            idx += 1
        idx = max(start_idx, min(idx, len(self.path_distances) - 2))
        
        # 计算在路径段上的位置
        prev_dist = self.path_distances[idx]
        next_dist = self.path_distances[idx + 1]
        segment_dist = next_dist - prev_dist
        
        if segment_dist < 0.001:  # 路径段过短
            return self.path_points[idx]
            
        ratio = (distance - prev_dist) / segment_dist
        x1, y1 = self.path_points[idx]
        x2, y2 = self.path_points[idx + 1]
        
        x = x1 + ratio * (x2 - x1)
        y = y1 + ratio * (y2 - y1)
        
        return (x, y)

    def get_entity_pose(self, entity_name):
        try:
            req = GetEntityState.Request()
            req.name = entity_name
            req.reference_frame = 'world'
            
            future = self.gazebo_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.01)
            
            if future.result() is None:
                return None
            res = future.result()
            return res.state.pose if res.success else None
        except Exception as e:
            if (self.error_counter % 100) == 0:
                self.get_logger().error(f'获取{entity_name}位置失败：{str(e)}')
            self.error_counter += 1
            return None

    def quaternion_to_yaw(self, x, y, z, w):
        """将四元数转换为yaw角（-π到+π）"""
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def calculate_speed_direction(self, prev_pos, curr_pos, time_diff):
        if not prev_pos or not curr_pos or time_diff < 0.01:
            return (0.0, (0.0, 0.0))  # (速度, 方向向量)
        
        dx = curr_pos.x - prev_pos.x
        dy = curr_pos.y - prev_pos.y
        distance = math.hypot(dx, dy)
        speed = distance / time_diff if time_diff > 0 else 0.0
        
        if distance < 0.001:
            direction = (0.0, 0.0)
        else:
            direction = (dx / distance, dy / distance)
            
        return (speed, direction)

    def calculate_center_distance(self, p1: Point, p2: Point) -> float:
        """计算中心点距离"""
        if not p1 or not p2:
            return float('inf')
        return math.hypot(p1.x - p2.x, p1.y - p2.y)

    def calculate_min_edge_distance(self, car_pos: Point, obs_pos: Point) -> float:
        """计算小车边缘到障碍物边缘的最小距离"""
        if not car_pos or not obs_pos:
            return None  # 位置无效时返回None
        
        # 计算中心距离
        center_dist = self.calculate_center_distance(car_pos, obs_pos)
        
        # 边缘距离 = 中心距离 - 两个半径之和
        return center_dist - (CAR_RADIUS + OBSTACLE_RADIUS)

    def calculate_line_intersection(self, seg1, seg2):
        """计算两条线段是否相交，返回交点坐标或None"""
        (x1, y1), (x2, y2) = seg1
        (x3, y3), (x4, y4) = seg2

        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denom == 0:
            return None  # 平行线

        t_num = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)
        u_num = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3))
        t = t_num / denom
        u = u_num / denom

        if 0 <= t <= 1 and 0 <= u <= 1:
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            return (x, y)
        return None

    def predict_obstacle_future_directions(self, obs_id, obs_pos, current_dir):
        """预测障碍物未来可能的运动方向，根据不同障碍物ID使用不同边界和运动轴"""
        if not obs_pos:
            return [current_dir]
        
        # 障碍物1：X轴水平运动
        if obs_id == 1:
            ox = obs_pos.x
            x_min, x_max = OBSTACLE1_X_MIN, OBSTACLE1_X_MAX
            future_dirs = [current_dir]
            
            # 接近左边界且向左移动 → 即将向右反转
            if ox <= x_min + REVERSE_THRESHOLD and current_dir < -0.1:
                future_dirs.append(1.0)  # 增加X正方向
            # 接近右边界且向右移动 → 即将向左反转
            if ox >= x_max - REVERSE_THRESHOLD and current_dir > 0.1:
                future_dirs.append(-1.0)  # 增加X负方向
        
        # 障碍物2：Y轴垂直运动
        else:  # obs_id == 2
            oy = obs_pos.y
            y_min, y_max = OBSTACLE2_Y_MIN, OBSTACLE2_Y_MAX
            future_dirs = [current_dir]  # current_dir为Y轴方向分量
            
            # 接近下边界且向下移动 → 即将向上反转
            if oy <= y_min + REVERSE_THRESHOLD and current_dir < -0.1:
                future_dirs.append(1.0)  # 增加Y正方向
            # 接近上边界且向上移动 → 即将向下反转
            if oy >= y_max - REVERSE_THRESHOLD and current_dir > 0.1:
                future_dirs.append(-1.0)  # 增加Y负方向
        
        return list(set(future_dirs))

    def update_red_zones(self):
        """更新所有障碍物的危险区域（障碍物2改为Y轴垂直方向）"""
        self.red_zones = []
        self.obstacle_center_lines = []  # 重置障碍物中心轨迹线
        
        # 处理障碍物1（X轴水平运动）
        if self.obs1_pos:
            ox, oy = self.obs1_pos.x, self.obs1_pos.y
            current_dir = self.obs1_direction[0]  # X轴方向分量
            future_dirs = self.predict_obstacle_future_directions(1, self.obs1_pos, current_dir)
            
            for dir_x in future_dirs:
                is_current_dir = (abs(dir_x - current_dir) < 0.1)
                if dir_x > 0.1:  # X正方向
                    x_start = ox - OBSTACLE_RADIUS  # 障碍物左边缘
                    x_end = OBSTACLE1_X_MAX + EXTENSION
                    y_min = oy - OBSTACLE_RADIUS - EXTENSION
                    y_max = oy + OBSTACLE_RADIUS + EXTENSION
                    self.red_zones.append( (x_start, x_end, y_min, y_max, is_current_dir, 1) )
                    # 记录障碍物中心轨迹线（水平中心线）
                    self.obstacle_center_lines.append(
                        [(x_start, oy), (x_end, oy), is_current_dir, 1]
                    )
                elif dir_x < -0.1:  # X负方向
                    x_start = ox + OBSTACLE_RADIUS  # 障碍物右边缘
                    x_end = OBSTACLE1_X_MIN - EXTENSION
                    y_min = oy - OBSTACLE_RADIUS - EXTENSION
                    y_max = oy + OBSTACLE_RADIUS + EXTENSION
                    self.red_zones.append( (x_start, x_end, y_min, y_max, is_current_dir, 1) )
                    # 记录障碍物中心轨迹线（水平中心线）
                    self.obstacle_center_lines.append(
                        [(x_start, oy), (x_end, oy), is_current_dir, 1]
                    )
        
        # 处理障碍物2（Y轴垂直运动）
        if self.obs2_pos:
            ox, oy = self.obs2_pos.x, self.obs2_pos.y
            current_dir = self.obs2_direction[1]  # Y轴方向分量
            future_dirs = self.predict_obstacle_future_directions(2, self.obs2_pos, current_dir)
            
            for dir_y in future_dirs:
                is_current_dir = (abs(dir_y - current_dir) < 0.1)
                if dir_y > 0.1:  # Y正方向（向上）
                    y_start = oy - OBSTACLE_RADIUS  # 障碍物下边缘
                    y_end = OBSTACLE2_Y_MAX + EXTENSION
                    x_min = ox - OBSTACLE_RADIUS - EXTENSION
                    x_max = ox + OBSTACLE_RADIUS + EXTENSION
                    self.red_zones.append( (x_min, x_max, y_start, y_end, is_current_dir, 2) )
                    # 记录Y轴方向中心轨迹线
                    self.obstacle_center_lines.append(
                        [(ox, y_start), (ox, y_end), is_current_dir, 2]
                    )
                elif dir_y < -0.1:  # Y负方向（向下）
                    y_start = oy + OBSTACLE_RADIUS  # 障碍物上边缘
                    y_end = OBSTACLE2_Y_MIN - EXTENSION
                    x_min = ox - OBSTACLE_RADIUS - EXTENSION
                    x_max = ox + OBSTACLE_RADIUS + EXTENSION
                    self.red_zones.append( (x_min, x_max, y_start, y_end, is_current_dir, 2) )
                    # 记录Y轴方向中心轨迹线
                    self.obstacle_center_lines.append(
                        [(ox, y_start), (ox, y_end), is_current_dir, 2]
                    )

    def check_path_intersection_with_zone(self, zone):
        """检查路径与危险区域的交点（计算逻辑不受显示模式影响）"""
        x1, x2, y1, y2, is_current, obs_id = zone
        red_segments = [
            [(x1, y1), (x2, y1)],  # 下边界
            [(x1, y2), (x2, y2)],  # 上边界
            [(x1, y1), (x1, y2)],  # 左边界
            [(x2, y1), (x2, y2)]   # 右边界
        ]

        # 查找对应的障碍物中心轨迹线
        center_segment = None
        for seg in self.obstacle_center_lines:
            line_seg, line_is_current, line_obs_id = seg[:2], seg[2], seg[3]
            if line_is_current == is_current and line_obs_id == obs_id:
                center_segment = line_seg
                break
        
        # 如果存在中心轨迹线，加入检测线段
        if center_segment:
            red_segments.append(center_segment)

        # 检测路径与所有危险区域线段的交点
        for i in range(len(self.path_points) - 1):
            path_seg = [self.path_points[i], self.path_points[i+1]]
            for red_seg in red_segments:
                intersection = self.calculate_line_intersection(path_seg, red_seg)
                if intersection:
                    # 计算交点与小车/障碍物的距离（考虑实体大小）
                    if obs_id == 1:
                        obs_point = Point(x=self.obs1_pos.x, y=self.obs1_pos.y, z=0.0) if self.obs1_pos else None
                    else:
                        obs_point = Point(x=self.obs2_pos.x, y=self.obs2_pos.y, z=0.0) if self.obs2_pos else None
                        
                    if not obs_point:
                        continue
                        
                    intersect_point = Point(x=intersection[0], y=intersection[1], z=0.0)
                    
                    # 计算障碍物到交点的距离（考虑障碍物大小）
                    dist_to_obs = self.calculate_center_distance(obs_point, intersect_point)
                    dist_to_obs -= OBSTACLE_RADIUS  # 修正：减去障碍物半径
                    
                    # 计算小车到交点的距离（考虑小车大小）
                    if self.car_pose:
                        car_point = self.car_pose.position
                        dist_to_car = self.calculate_center_distance(car_point, intersect_point)
                        dist_to_car -= CAR_RADIUS  # 修正：减去小车半径
                    else:
                        dist_to_car = float('inf')
                        
                    return True, dist_to_obs, dist_to_car, intersection, obs_id
        return False, 0.0, 0.0, None, None

    def predict_future_positions(self):
        """预测未来位置"""
        self.predicted_car_positions = []
        self.predicted_obs_positions = {1: [], 2: []}  # 重置预测位置
        self.collision_prediction = None
        
        if not self.car_pose or not self.path_points or len(self.path_points) < 2:
            return
            
        # 当前位置和运动参数
        car_pos = self.car_pose.position
        car_speed = self.car_speed if self.car_speed > MIN_SPEED_THRESHOLD else 0.0
        
        # 找到小车在导航路径上的位置和累积距离
        self.car_path_index, closest_point = self.find_closest_point_on_path(car_pos.x, car_pos.y)
        if self.car_path_index >= len(self.path_distances):
            self.car_path_index = len(self.path_distances) - 1
        current_dist = self.path_distances[self.car_path_index]
        
        # 预测小车位置（沿导航轨迹）
        for i in range(1, PREDICTION_STEPS + 1):
            t = i * PREDICTION_STEP
            predicted_distance = current_dist + car_speed * t
            pred_pos = self.get_position_along_path(self.car_path_index, predicted_distance)
            
            if pred_pos:
                self.predicted_car_positions.append((pred_pos[0], pred_pos[1], t))
            else:
                if self.predicted_car_positions:
                    last_pos = self.predicted_car_positions[-1]
                    self.predicted_car_positions.append((last_pos[0], last_pos[1], t))
                else:
                    self.predicted_car_positions.append((car_pos.x, car_pos.y, t))
        
        # 预测障碍物1位置（X轴水平运动，考虑边界反弹）
        if self.obs1_pos:
            obs_pos = self.obs1_pos
            obs_speed = self.obs1_speed if self.obs1_speed > MIN_SPEED_THRESHOLD else 0.0
            obs_dir_x, obs_dir_y = self.obs1_direction
            
            for i in range(1, PREDICTION_STEPS + 1):
                t = i * PREDICTION_STEP
                pred_obs_x = obs_pos.x
                pred_obs_y = obs_pos.y
                remaining_time = t
                
                current_obs_dir_x, current_obs_dir_y = obs_dir_x, obs_dir_y
                temp_obs_x, temp_obs_y = pred_obs_x, pred_obs_y
                
                if obs_speed < MIN_SPEED_THRESHOLD:
                    pred_obs_x, pred_obs_y = temp_obs_x, temp_obs_y
                    self.predicted_obs_positions[1].append((pred_obs_x, pred_obs_y, t))
                    continue
                    
                while remaining_time > 0:
                    if abs(current_obs_dir_x) < 0.01:
                        time_to_boundary = float('inf')
                    elif current_obs_dir_x > 0:
                        distance_to_boundary = (OBSTACLE1_X_MAX - OBSTACLE_RADIUS) - temp_obs_x
                        time_to_boundary = distance_to_boundary / (current_obs_dir_x * obs_speed) if distance_to_boundary > 0 else float('inf')
                    else:
                        distance_to_boundary = temp_obs_x - (OBSTACLE1_X_MIN + OBSTACLE_RADIUS)
                        time_to_boundary = distance_to_boundary / (-current_obs_dir_x * obs_speed) if distance_to_boundary > 0 else float('inf')
                    
                    time_to_boundary = max(time_to_boundary, 0.0)
                    move_time = min(time_to_boundary, remaining_time)
                    
                    temp_obs_x += current_obs_dir_x * obs_speed * move_time
                    temp_obs_y += current_obs_dir_y * obs_speed * move_time
                    remaining_time -= move_time
                    
                    if abs(remaining_time) < 1e-6:
                        break
                        
                    if current_obs_dir_x > 0 and temp_obs_x >= OBSTACLE1_X_MAX - OBSTACLE_RADIUS - 1e-6:
                        current_obs_dir_x = -current_obs_dir_x
                    elif current_obs_dir_x < 0 and temp_obs_x <= OBSTACLE1_X_MIN + OBSTACLE_RADIUS + 1e-6:
                        current_obs_dir_x = -current_obs_dir_x
                
                self.predicted_obs_positions[1].append((temp_obs_x, temp_obs_y, t))
        
        # 预测障碍物2位置（Y轴垂直运动，考虑上下边界反弹）
        if self.obs2_pos:
            obs_pos = self.obs2_pos
            obs_speed = self.obs2_speed if self.obs2_speed > MIN_SPEED_THRESHOLD else 0.0
            obs_dir_x, obs_dir_y = self.obs2_direction  # Y轴方向分量为主
            
            for i in range(1, PREDICTION_STEPS + 1):
                t = i * PREDICTION_STEP
                pred_obs_x = obs_pos.x  # X坐标不变
                pred_obs_y = obs_pos.y
                remaining_time = t
                
                current_obs_dir_y = obs_dir_y  # 重点关注Y轴方向
                temp_obs_x, temp_obs_y = pred_obs_x, pred_obs_y
                
                if obs_speed < MIN_SPEED_THRESHOLD:
                    self.predicted_obs_positions[2].append((pred_obs_x, pred_obs_y, t))
                    continue
                    
                while remaining_time > 0:
                    if abs(current_obs_dir_y) < 0.01:
                        time_to_boundary = float('inf')
                    elif current_obs_dir_y > 0:  # 向上运动
                        distance_to_boundary = (OBSTACLE2_Y_MAX - OBSTACLE_RADIUS) - temp_obs_y
                        time_to_boundary = distance_to_boundary / (current_obs_dir_y * obs_speed) if distance_to_boundary > 0 else float('inf')
                    else:  # 向下运动
                        distance_to_boundary = temp_obs_y - (OBSTACLE2_Y_MIN + OBSTACLE_RADIUS)
                        time_to_boundary = distance_to_boundary / (-current_obs_dir_y * obs_speed) if distance_to_boundary > 0 else float('inf')
                    
                    time_to_boundary = max(time_to_boundary, 0.0)
                    move_time = min(time_to_boundary, remaining_time)
                    
                    temp_obs_x += obs_dir_x * obs_speed * move_time  # X方向无运动
                    temp_obs_y += current_obs_dir_y * obs_speed * move_time  # 仅Y方向移动
                    remaining_time -= move_time
                    
                    if abs(remaining_time) < 1e-6:
                        break
                        
                    # 触碰边界反转Y轴方向
                    if current_obs_dir_y > 0 and temp_obs_y >= OBSTACLE2_Y_MAX - OBSTACLE_RADIUS - 1e-6:
                        current_obs_dir_y = -current_obs_dir_y
                    elif current_obs_dir_y < 0 and temp_obs_y <= OBSTACLE2_Y_MIN + OBSTACLE_RADIUS + 1e-6:
                        current_obs_dir_y = -current_obs_dir_y
                
                self.predicted_obs_positions[2].append((temp_obs_x, temp_obs_y, t))
        
        # 检查预测轨迹是否有碰撞
        min_collision_time = float('inf')
        for obs_id in [1, 2]:
            for i in range(len(self.predicted_car_positions)):
                if i >= len(self.predicted_obs_positions[obs_id]):
                    break
                    
                car_x, car_y, t = self.predicted_car_positions[i]
                obs_x, obs_y, _ = self.predicted_obs_positions[obs_id][i]
                
                dist = math.hypot(car_x - obs_x, car_y - obs_y)
                if dist < (CAR_RADIUS + OBSTACLE_RADIUS - 1e-6) and t < min_collision_time:
                    self.collision_prediction = (t, car_x, car_y, obs_id)
                    min_collision_time = t

    def check_collision_risk(self):
        """检查是否存在碰撞风险"""
        self.collision_point = None
        self.time_diff = None
        self.edge_distance = None
        self.colliding_obstacle = None
        
        # 首先检查预测轨迹是否有碰撞
        if self.collision_prediction:
            t, x, y, obs_id = self.collision_prediction
            self.collision_point = (x, y)
            self.time_diff = t
            self.colliding_obstacle = obs_id
            return True
            
        # 检查当前状态下的碰撞风险
        if not self.path_points or not self.red_zones or not self.car_pose:
            return False
        
        car_pos = self.car_pose.position
        car_speed = self.car_speed if self.car_speed > MIN_SPEED_THRESHOLD else 0.0
        car_dir_x = math.cos(self.car_yaw)  # 小车x方向分量

        # 遍历所有危险区域，检查是否有碰撞风险
        min_time_diff = float('inf')
        closest_collision_point = None
        closest_edge_distance = None
        closest_obs_id = None
        
        for zone in self.red_zones:
            has_intersection, dist_to_obs, dist_to_car, intersection, obs_id = self.check_path_intersection_with_zone(zone)
            if not has_intersection:
                continue

            if dist_to_obs > OBSTACLE_DISTANCE_THRESHOLD or dist_to_car < 0:
                continue

            # 获取对应障碍物的位置和速度
            if obs_id == 1 and self.obs1_pos:
                obs_pos = self.obs1_pos
                obs_speed = self.obs1_speed if self.obs1_speed > MIN_SPEED_THRESHOLD else 0.0
            elif obs_id == 2 and self.obs2_pos:
                obs_pos = self.obs2_pos
                obs_speed = self.obs2_speed if self.obs2_speed > MIN_SPEED_THRESHOLD else 0.0
            else:
                continue

            # 计算边缘距离
            edge_distance = self.calculate_min_edge_distance(car_pos, obs_pos)
            if edge_distance is None:
                continue

            # 边缘距离小于安全阈值 → 存在碰撞风险
            if edge_distance < SAFE_EDGE_DISTANCE:
                intersect_x, intersect_y = intersection
                if obs_id == 1:
                    # 障碍物1：X轴方向计算时间
                    zone_dir = 1.0 if zone[0] < zone[1] else -1.0
                    t_obs = self.calculate_time_to_point(obs_pos, obs_speed, zone_dir, intersect_x)
                    t_car = self.calculate_time_to_point(car_pos, car_speed, car_dir_x, intersect_x)
                else:
                    # 障碍物2：Y轴方向计算时间
                    zone_dir = 1.0 if zone[2] < zone[3] else -1.0
                    t_obs = self.calculate_time_to_point_y(obs_pos, obs_speed, zone_dir, intersect_y)
                    t_car = self.calculate_time_to_point_y(car_pos, car_speed, math.sin(self.car_yaw), intersect_y)
                
                time_diff = abs(t_car - t_obs)
                if time_diff < SAFE_TIME_THRESHOLD and time_diff < min_time_diff:
                    min_time_diff = time_diff
                    closest_collision_point = (intersect_x, intersect_y)
                    closest_edge_distance = edge_distance
                    closest_obs_id = obs_id
        
        if closest_obs_id is not None:
            self.collision_point = closest_collision_point
            self.time_diff = min_time_diff
            self.edge_distance = closest_edge_distance
            self.colliding_obstacle = closest_obs_id
            return True
        
        return False

    def calculate_time_to_point(self, entity_pos, entity_speed, entity_dir, target_x):
        """计算实体沿X轴到达目标点的时间（适配障碍物1水平运动）"""
        if abs(entity_speed) < MIN_SPEED_THRESHOLD:
            return float('inf')
        dx = target_x - entity_pos.x
        if (dx > 0 and entity_dir > 0) or (dx < 0 and entity_dir < 0):
            return abs(dx) / entity_speed
        else:
            return float('inf')

    def calculate_time_to_point_y(self, entity_pos, entity_speed, entity_dir, target_y):
        """计算实体沿Y轴到达目标点的时间（适配障碍物2垂直运动）"""
        if abs(entity_speed) < MIN_SPEED_THRESHOLD:
            return float('inf')
        dy = target_y - entity_pos.y
        if (dy > 0 and entity_dir > 0) or (dy < 0 and entity_dir < 0):
            return abs(dy) / entity_speed
        else:
            return float('inf')

    def set_controller_vel_params(self, is_stop):
        target_params = self.stop_vel_params if is_stop else self.initial_vel_params
        params_list = []
        for param_name, param_value in target_params.items():
            full_param_name = f"FollowPath.{param_name}"
            param = Parameter()
            param.name = full_param_name
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = param_value
            params_list.append(param)

        req = SetParameters.Request()
        req.parameters = params_list
        future = self.param_client.call_async(req)
        status = "停止" if is_stop else "恢复"

        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.result() is None:
            self.get_logger().error(f"[失败] {status}控制器速度限制：服务调用超时")
            return
        
        all_successful = all(result.successful for result in future.result().results)
        if all_successful:
            self.get_logger().warning(f"[成功] {status}控制器速度限制")
            self.is_stopped = is_stop
        else:
            failed_params = [
                param.name for param, result in zip(params_list, future.result().results)
                if not result.successful
            ]
            self.get_logger().error(f"[失败] 以下参数修改失败：{failed_params}")

    def update_entity_data(self):
        current_time = time.time()
        time_diff = current_time - self.prev_update_time
        
        # 保存之前的位置
        self.car_prev_pose = self.car_pose
        self.obs1_prev_pos = self.obs1_pos if self.obs1_pos else None
        self.obs2_prev_pos = self.obs2_pos if self.obs2_pos else None

        # 获取当前位置和姿态
        self.car_pose = self.get_entity_pose('six_arm')
        obs1_pose = self.get_entity_pose('Move1')
        obs2_pose = self.get_entity_pose('Move2')
        
        self.obs1_pos = obs1_pose.position if obs1_pose else None
        self.obs2_pos = obs2_pose.position if obs2_pose else None
        
        # 记录运动轨迹
        if self.car_pose:
            car_pos = self.car_pose.position
            self.last_known_car_pos = (car_pos.x, car_pos.y)
            self.car_trail.append((car_pos.x, car_pos.y))
            if len(self.car_trail) > TRAIL_LENGTH:
                self.car_trail.pop(0)
                
        if self.obs1_pos:
            self.obs_trail[1].append((self.obs1_pos.x, self.obs1_pos.y))
            if len(self.obs_trail[1]) > TRAIL_LENGTH:
                self.obs_trail[1].pop(0)
                
        if self.obs2_pos:
            self.obs_trail[2].append((self.obs2_pos.x, self.obs2_pos.y))
            if len(self.obs_trail[2]) > TRAIL_LENGTH:
                self.obs_trail[2].pop(0)
        
        # 提取小车yaw角
        if self.car_pose:
            orientation = self.car_pose.orientation
            self.car_yaw = self.quaternion_to_yaw(
                orientation.x, orientation.y, orientation.z, orientation.w
            )

        # 计算速度和方向
        car_pos = self.car_pose.position if self.car_pose else None
        self.car_speed, self.car_direction = self.calculate_speed_direction(
            self.car_prev_pose.position if self.car_prev_pose else None,
            car_pos,
            time_diff
        )

        self.obs1_speed, self.obs1_direction = self.calculate_speed_direction(
            self.obs1_prev_pos,
            self.obs1_pos,
            time_diff
        )
        
        self.obs2_speed, self.obs2_direction = self.calculate_speed_direction(
            self.obs2_prev_pos,
            self.obs2_pos,
            time_diff
        )

        # 更新红色警戒区域和障碍物中心轨迹线
        self.update_red_zones()
        
        # 预测未来位置
        self.predict_future_positions()

        # 检查碰撞条件
        stop_condition_met = self.check_collision_risk()

        # 滞后判断
        if stop_condition_met:
            need_stop = True
        else:
            path_in_any_zone = any(self.check_path_intersection_with_zone(zone)[0] for zone in self.red_zones)
            need_stop = path_in_any_zone and self.is_stopped

        # 执行停止/恢复操作
        if need_stop and not self.is_stopped:
            self.set_controller_vel_params(is_stop=True)
        elif not need_stop and self.is_stopped:
            self.set_controller_vel_params(is_stop=False)

        # 定期日志输出
        if car_pos and (self.log_counter % 10 == 0):
            self.get_logger().info('\n==========================')
            self.get_logger().info(f'小车位置：({car_pos.x:.2f}, {car_pos.y:.2f}) 速度：{self.car_speed:.2f}m/s')
            if self.obs1_pos:
                self.get_logger().info(f'障碍物1位置：({self.obs1_pos.x:.2f}, {self.obs1_pos.y:.2f}) 速度：{self.obs1_speed:.2f}m/s')
            if self.obs2_pos:
                self.get_logger().info(f'障碍物2位置：({self.obs2_pos.x:.2f}, {self.obs2_pos.y:.2f}) 速度：{self.obs2_speed:.2f}m/s')
            
            if self.collision_prediction:
                t, x, y, obs_id = self.collision_prediction
                self.get_logger().info(f'预测与障碍物{obs_id}碰撞时间：{t:.1f}s 在位置：({x:.2f}, {y:.2f})')
            elif self.time_diff is not None and self.colliding_obstacle is not None:
                self.get_logger().info(f'与障碍物{self.colliding_obstacle}的碰撞点：{self.collision_point if self.collision_point else "无"} 时间差：{self.time_diff:.2f}s')
            else:
                self.get_logger().info(f'无碰撞风险')
            
            self.get_logger().info(f'停止状态：{"已停止" if self.is_stopped else "正常行驶"}')

        self.prev_update_time = current_time
        self.log_counter = (self.log_counter + 1) % 1000

    def update_plot(self):
        # 显示模式判断
        hide_red_zones_and_center_lines = (SHOW_MODE == 0)
        
        # 更新导航路径（始终显示）
        if self.path_points:
            x, y = zip(*self.path_points)
            self.path_line.set_data(x, y)
            self.path_line.set_visible(True)
        else:
            self.path_line.set_visible(False)
        
        # 更新小车
        car_pos = self.car_pose.position if self.car_pose else None
        if car_pos:
            # 小车位置与安全圈（始终显示）
            self.car_circle.set_center((car_pos.x, car_pos.y))
            self.safe_distance_circle.set_center((car_pos.x, car_pos.y))
            self.safe_distance_circle.set_visible(True)
            
            # 速度箭头（始终显示）
            self.car_dir_arrow.remove()
            arrow_len = min(0.8, self.car_speed * 0.5)
            dx = math.cos(self.car_yaw) * arrow_len
            dy = math.sin(self.car_yaw) * arrow_len
            self.car_dir_arrow = plt.Arrow(
                car_pos.x, car_pos.y, dx, dy, width=0.15, color='blue'
            )
            self.ax.add_patch(self.car_dir_arrow)
            
            # 小车轨迹（始终显示）
            if self.car_trail:
                tx, ty = zip(*self.car_trail)
                self.car_trail_line.set_data(tx, ty)
                self.car_trail_line.set_visible(True)
            else:
                self.car_trail_line.set_visible(False)
        else:
            # 位置获取失败时，显示最后已知位置
            self.car_circle.set_center(self.last_known_car_pos)
            self.safe_distance_circle.set_center(self.last_known_car_pos)
            self.car_trail_line.set_visible(False)

        # 更新障碍物1（始终显示）
        if self.obs1_pos:
            self.obs1_circle.set_center((self.obs1_pos.x, self.obs1_pos.y))
            self.obs1_circle.set_radius(OBSTACLE_RADIUS)
            
            # 速度箭头（始终显示）
            self.obs1_dir_arrow.remove()
            arrow_len = min(0.8, self.obs1_speed * 0.5)
            dx = self.obs1_direction[0] * arrow_len
            dy = self.obs1_direction[1] * arrow_len
            self.obs1_dir_arrow = plt.Arrow(
                self.obs1_pos.x, self.obs1_pos.y, dx, dy, width=0.15, color='green'
            )
            self.ax.add_patch(self.obs1_dir_arrow)
            
            # 障碍物轨迹（始终显示）
            if self.obs_trail[1]:
                tx, ty = zip(*self.obs_trail[1])
                self.obs1_trail_line.set_data(tx, ty)
                self.obs1_trail_line.set_visible(True)
            else:
                self.obs1_trail_line.set_visible(False)
        else:
            self.obs1_circle.set_radius(0)
            self.obs1_trail_line.set_visible(False)
            
        # 更新障碍物2（始终显示）
        if self.obs2_pos:
            self.obs2_circle.set_center((self.obs2_pos.x, self.obs2_pos.y))
            self.obs2_circle.set_radius(OBSTACLE_RADIUS)
            
            # 速度箭头（始终显示）
            self.obs2_dir_arrow.remove()
            arrow_len = min(0.8, self.obs2_speed * 0.5)
            dx = self.obs2_direction[0] * arrow_len
            dy = self.obs2_direction[1] * arrow_len
            self.obs2_dir_arrow = plt.Arrow(
                self.obs2_pos.x, self.obs2_pos.y, dx, dy, width=0.15, color='purple'
            )
            self.ax.add_patch(self.obs2_dir_arrow)
            
            # 障碍物轨迹（始终显示）
            if self.obs_trail[2]:
                tx, ty = zip(*self.obs_trail[2])
                self.obs2_trail_line.set_data(tx, ty)
                self.obs2_trail_line.set_visible(True)
            else:
                self.obs2_trail_line.set_visible(False)
        else:
            self.obs2_circle.set_radius(0)
            self.obs2_trail_line.set_visible(False)

        # 更新障碍物中心轨迹线（根据模式控制显示）
        if not hide_red_zones_and_center_lines and self.obstacle_center_lines:
            # 障碍物1的中心轨迹线
            obs1_lines = [seg[:2] for seg in self.obstacle_center_lines if seg[3] == 1 and seg[2]]
            if obs1_lines:
                x1_data = []
                y1_data = []
                for (p1, p2) in obs1_lines:
                    x1_data.extend([p1[0], p2[0]])
                    y1_data.extend([p1[1], p2[1]])
                self.obstacle1_center_line.set_data(x1_data, y1_data)
                self.obstacle1_center_line.set_visible(True)
            else:
                self.obstacle1_center_line.set_visible(False)
                
            # 障碍物2的中心轨迹线
            obs2_lines = [seg[:2] for seg in self.obstacle_center_lines if seg[3] == 2 and seg[2]]
            if obs2_lines:
                x2_data = []
                y2_data = []
                for (p1, p2) in obs2_lines:
                    x2_data.extend([p1[0], p2[0]])
                    y2_data.extend([p1[1], p2[1]])
                self.obstacle2_center_line.set_data(x2_data, y2_data)
                self.obstacle2_center_line.set_visible(True)
            else:
                self.obstacle2_center_line.set_visible(False)
        else:
            self.obstacle1_center_line.set_visible(False)
            self.obstacle2_center_line.set_visible(False)

        # 更新危险区域（根据模式控制显示）
        if not hide_red_zones_and_center_lines:
            current_idx = 0
            future_idx = 0
            for zone in self.red_zones:
                x1, x2, y1, y2, is_current, _ = zone
                width = abs(x2 - x1)
                height = abs(y2 - y1)
                x_min = min(x1, x2)
                y_min = min(y1, y2)
                
                if is_current and current_idx < len(self.red_zone_patches):
                    self.red_zone_patches[current_idx].set_xy((x_min, y_min))
                    self.red_zone_patches[current_idx].set_width(width)
                    self.red_zone_patches[current_idx].set_height(height)
                    current_idx += 1
                elif not is_current and future_idx < len(self.future_red_zone_patches):
                    self.future_red_zone_patches[future_idx].set_xy((x_min, y_min))
                    self.future_red_zone_patches[future_idx].set_width(width)
                    self.future_red_zone_patches[future_idx].set_height(height)
                    future_idx += 1
        else:
            # 隐藏红色危险区域
            for patch in self.red_zone_patches + self.future_red_zone_patches:
                patch.set_width(0)
                patch.set_height(0)

        # 更新预测位置（始终显示）
        for i in range(PREDICTION_STEPS):
            # 小车预测位置
            if i < len(self.predicted_car_positions):
                x, y, _ = self.predicted_car_positions[i]
                self.predicted_car_markers[i].set_center((x, y))
                self.predicted_car_markers[i].set_radius(CAR_RADIUS * 0.8)
                alpha = 0.3 - (i / PREDICTION_STEPS) * 0.2
                self.predicted_car_markers[i].set_alpha(max(0.1, alpha))
                self.predicted_car_markers[i].set_visible(True)
            else:
                self.predicted_car_markers[i].set_visible(False)
                
            # 障碍物1预测位置
            if i < len(self.predicted_obs_positions[1]):
                x, y, _ = self.predicted_obs_positions[1][i]
                self.predicted_obs1_markers[i].set_center((x, y))
                self.predicted_obs1_markers[i].set_radius(OBSTACLE_RADIUS * 0.8)
                alpha = 0.3 - (i / PREDICTION_STEPS) * 0.2
                self.predicted_obs1_markers[i].set_alpha(max(0.1, alpha))
                self.predicted_obs1_markers[i].set_visible(True)
            else:
                self.predicted_obs1_markers[i].set_visible(False)
                
            # 障碍物2预测位置
            if i < len(self.predicted_obs_positions[2]):
                x, y, _ = self.predicted_obs_positions[2][i]
                self.predicted_obs2_markers[i].set_center((x, y))
                self.predicted_obs2_markers[i].set_radius(OBSTACLE_RADIUS * 0.8)
                alpha = 0.3 - (i / PREDICTION_STEPS) * 0.2
                self.predicted_obs2_markers[i].set_alpha(max(0.1, alpha))
                self.predicted_obs2_markers[i].set_visible(True)
            else:
                self.predicted_obs2_markers[i].set_visible(False)

        # 更新碰撞风险标记（始终显示）
        if self.collision_prediction:
            t, x, y, obs_id = self.collision_prediction
            self.collision_marker.set_data([x], [y])
            self.collision_marker.set_visible(True)
            
            self.time_diff_text.set_position((x + 0.2, y + 0.2))
            self.time_diff_text.set_text(f"Collision Time: {t:.1f}s")
            self.time_diff_text.set_color('red')
            self.time_diff_text.set_visible(True)
            
        elif self.collision_point and self.edge_distance is not None and self.colliding_obstacle is not None:
            self.collision_marker.set_data([self.collision_point[0]], [self.collision_point[1]])
            self.collision_marker.set_visible(True)
            
            self.time_diff_text.set_position((self.collision_point[0]+0.2, self.collision_point[1]+0.2))
            self.time_diff_text.set_text(f"{self.colliding_obstacle} Δt: {self.time_diff:.2f}s")
            self.time_diff_text.set_color('red' if self.time_diff < SAFE_TIME_THRESHOLD else 'green')
            self.time_diff_text.set_visible(True)
            
            self.edge_distance_text.set_position((self.collision_point[0]+0.2, self.collision_point[1]-0.3))
            self.edge_distance_text.set_text(f"Edge Distance: {self.edge_distance:.2f}m")
            self.edge_distance_text.set_color('red' if self.edge_distance < SAFE_EDGE_DISTANCE else 'green')
            self.edge_distance_text.set_visible(True)
        else:
            self.collision_marker.set_data([], [])
            self.collision_marker.set_visible(False)
            self.time_diff_text.set_text('')
            self.time_diff_text.set_visible(False)
            self.edge_distance_text.set_text('')
            self.edge_distance_text.set_visible(False)

        # 调整坐标轴范围
        all_points = []
        if self.path_points:
            all_points.extend(self.path_points)
        if car_pos:
            all_points.append((car_pos.x, car_pos.y))
        else:
            all_points.append(self.last_known_car_pos)
        if self.obs1_pos:
            all_points.append((self.obs1_pos.x, self.obs1_pos.y))
        if self.obs2_pos:
            all_points.append((self.obs2_pos.x, self.obs2_pos.y))
        if self.collision_point:
            all_points.append(self.collision_point)
        if self.collision_prediction:
            t, x, y, _ = self.collision_prediction
            all_points.append((x, y))
        # 即使隐藏危险区域，计算范围时仍需考虑以保证视图正确
        for zone in self.red_zones:
            x1, x2, y1, y2, _, _ = zone
            all_points.append((x1, y1))
            all_points.append((x2, y2))
        for line in self.obstacle_center_lines:
            (x1, y1), (x2, y2), _, _ = line
            all_points.append((x1, y1))
            all_points.append((x2, y2))
        all_points.extend(self.car_trail)
        all_points.extend(self.obs_trail[1])
        all_points.extend(self.obs_trail[2])
        all_points.extend([(x, y) for x, y, t in self.predicted_car_positions])
        all_points.extend([(x, y) for x, y, t in self.predicted_obs_positions[1]])
        all_points.extend([(x, y) for x, y, t in self.predicted_obs_positions[2]])

        if all_points:
            xs, ys = zip(*all_points)
            self.ax.set_xlim(min(xs) - 1, max(xs) + 1)
            self.ax.set_ylim(min(ys) - 1, max(ys) + 1)

        self.canvas.draw()

    def update_all(self):
        rclpy.spin_once(self, timeout_sec=0.001)
        self.update_entity_data()
        self.update_plot()
        self.root.after(UPDATE_INTERVAL, self.update_all)

def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    cas = CollisionAvoidanceSystem(root)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        cas.get_logger().info('程序停止')
    finally:
        cas.set_controller_vel_params(is_stop=False)
        cas.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
