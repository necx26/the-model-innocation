import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from datetime import datetime
import os
import threading
import numpy as np

# 无需设置中文显示
# 移除中文显示相关配置

class VelocityRecorder(Node):
    def __init__(self):
        super().__init__('velocity_recorder')
        
        # 订阅/cmd_vel话题
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量警告
        
        # 初始化数据存储列表
        self.times = []
        self.linear_x = []
        self.linear_y = []
        self.linear_z = []
        self.angular_x = []
        self.angular_y = []
        self.angular_z = []
        
        # 记录开始时间
        self.start_time = datetime.now()
        
        # 创建保存数据的目录
        self.save_dir = "velocity_data"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        self.get_logger().info('速度记录器已启动，关闭图形窗口或按Ctrl+C停止并保存数据')

    def listener_callback(self, msg):
        """回调函数，处理接收到的速度数据"""
        # 计算相对时间
        current_time = (datetime.now() - self.start_time).total_seconds()
        
        # 存储数据
        self.times.append(current_time)
        self.linear_x.append(msg.linear.x)
        self.linear_y.append(msg.linear.y)
        self.linear_z.append(msg.linear.z)
        self.angular_x.append(msg.angular.x)
        self.angular_y.append(msg.angular.y)
        self.angular_z.append(msg.angular.z)

    def save_data(self):
        """保存记录的数据到文件"""
        if len(self.times) == 0:
            self.get_logger().warning('没有数据可保存')
            return
            
        # 生成带时间戳的文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.save_dir, f'velocity_data_{timestamp}.csv')
        
        # 保存为CSV文件
        with open(filename, 'w') as f:
            f.write('时间(s),线速度x(m/s),线速度y(m/s),线速度z(m/s),角速度x(rad/s),角速度y(rad/s),角速度z(rad/s)\n')
            for i in range(len(self.times)):
                f.write(f'{self.times[i]:.4f},{self.linear_x[i]:.6f},{self.linear_y[i]:.6f},{self.linear_z[i]:.6f},'
                        f'{self.angular_x[i]:.6f},{self.angular_y[i]:.6f},{self.angular_z[i]:.6f}\n')
        
        self.get_logger().info(f'数据已保存到: {filename}')


def ros_spin_thread(node):
    """在子线程中运行ROS的spin"""
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


def main(args=None):
    rclpy.init(args=args)
    velocity_recorder = VelocityRecorder()
    
    # 在子线程中运行ROS节点
    ros_thread = threading.Thread(target=ros_spin_thread, args=(velocity_recorder,), daemon=True)
    ros_thread.start()
    
    # 创建单个图形（在主线程中）
    fig, ax = plt.subplots(figsize=(12, 8))
    fig.suptitle('Robot Velocity Components Monitoring', fontsize=16)  # 英文标题
    
    # 设置坐标轴标签（英文）
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity Value (Linear: m/s, Angular: rad/s)')
    ax.grid(True, linestyle='--', alpha=0.7)
    
    # 定义每个分量的样式（颜色和线型），标签改为英文
    styles = [
        ('Linear Velocity x', 'r-', 'red'),    # 红色实线
        ('Linear Velocity y', 'g--', 'green'),  # 绿色虚线
        ('Linear Velocity z', 'b-.', 'blue'),   # 蓝色点划线
        ('Angular Velocity x', 'c:', 'cyan'),    # 青色点线
        ('Angular Velocity y', 'm-', 'magenta'), # 品红实线
        ('Angular Velocity z', 'y--', 'yellow')  # 黄色虚线
    ]
    
    # 初始化线条
    lines = []
    for label, style, _ in styles:
        line, = ax.plot([], [], style, label=label, linewidth=1.5)
        lines.append(line)
    
    # 添加图例
    ax.legend(loc='upper right')
    
    def update_plot(frame):
        """更新图形的回调函数，实现纵轴自适应调整"""
        if len(velocity_recorder.times) == 0:
            return lines
        
        # 准备要绘制的数据
        data = [
            velocity_recorder.linear_x,
            velocity_recorder.linear_y,
            velocity_recorder.linear_z,
            velocity_recorder.angular_x,
            velocity_recorder.angular_y,
            velocity_recorder.angular_z
        ]
        
        # 更新每条线的数据
        for i, line in enumerate(lines):
            line.set_data(velocity_recorder.times, data[i])
        
        # 计算所有数据的最大值和最小值，用于自适应纵轴
        all_data = []
        for d in data:
            all_data.extend(d)
        
        if all_data:  # 确保数据不为空
            # 计算数据范围，增加10%的边距避免曲线贴边
            data_min = min(all_data)
            data_max = max(all_data)
            margin = (data_max - data_min) * 0.1  # 10%边距
            
            # 处理所有数据相同的特殊情况（避免除以零）
            if data_max == data_min:
                margin = 0.1  # 固定边距
            
            ax.set_ylim(data_min - margin, data_max + margin)
        
        # 自动调整横轴范围
        ax.relim()
        ax.autoscale_view(scalex=True, scaley=False)  # 仅横轴自动调整
        
        return lines
    
    # 设置动画
    ani = FuncAnimation(
        fig, 
        update_plot, 
        interval=100,  # 每100ms更新一次
        blit=True
    )
    
    try:
        # 在主线程中显示图形
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        # 保存数据并清理
        velocity_recorder.save_data()
        velocity_recorder.destroy_node()
        rclpy.shutdown()
        ros_thread.join()


if __name__ == '__main__':
    main()
