import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Quaternion
import math
import time

class ModelController(Node):
    def __init__(self):
        super().__init__('move_gazebo_models')
        # 创建服务客户端
        self.client = self.create_client(SetModelState, '/gazebo/set_model_state')
        
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，等待中...')
        
        # 初始化变量
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        
        # 两个模型的初始位置（基于world文件中的初始pose）
        self.models = {
            'Move1': {'x': -2.00001, 'y': -1.00016, 'z': 0.909362},
            'Move2': {'x': 1.99995, 'y': -1.2e-05, 'z': 0.909362}
        }
        
        self.n_line = 0
        self.do_skew = True

    def set_model_state(self, model_name, x, y, z):
        """设置单个模型的状态"""
        req = SetModelState.Request()
        model_state = ModelState()
        
        model_state.model_name = model_name
        model_state.reference_frame = "world"
        
        # 设置位置
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        # 设置姿态（单位四元数）
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = 0.0
        quat.w = 1.0
        pose.orientation = quat
        
        model_state.pose = pose
        model_state.twist = self.twist
        
        req.model_state = model_state
        future = self.client.call_async(req)
        
        # 等待服务响应
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"模型 {model_name} 位置已更新: ({x:.2f}, {y:.2f}, {z:.2f})")
        else:
            self.get_logger().error(f"设置模型 {model_name} 状态失败")

    def run(self):
        """主控制循环"""
        while rclpy.ok():
            # 计算新位置（与C++代码相同的运动逻辑）
            for model in self.models:
                if self.n_line == 0:
                    self.models[model]['x'] += 0.1
                    self.models[model]['y'] = self.models[model]['y']  # y保持不变
                    self.models[model]['z'] = 0.25
                    if self.models[model]['x'] >= 10:
                        self.n_line = 1
                elif self.n_line == 1:
                    self.models[model]['x'] = self.models[model]['x']  # x保持不变
                    self.models[model]['y'] += 0.1
                    self.models[model]['z'] = 0.25
                    if self.models[model]['y'] >= 10:
                        self.n_line = 2
                elif self.n_line == 2:
                    self.models[model]['x'] -= 0.1
                    self.models[model]['y'] = self.models[model]['y']  # y保持不变
                    self.models[model]['z'] = 0.25
                    if self.models[model]['x'] <= -10:
                        self.n_line = 3
                elif self.n_line == 3:
                    self.models[model]['x'] = self.models[model]['x']  # x保持不变
                    self.models[model]['y'] -= 0.1
                    self.models[model]['z'] = 0.25
                    if self.models[model]['y'] <= -10:
                        self.n_line = 4
                elif self.n_line == 4:
                    self.models[model]['x'] += 0.1
                    self.models[model]['y'] = self.models[model]['y']  # y保持不变
                    self.models[model]['z'] = 0.25
                    if self.models[model]['x'] >= 10:
                        self.n_line = 5
                elif self.n_line == 5:
                    self.models[model]['x'] = self.models[model]['x']  # x保持不变
                    self.models[model]['y'] += 0.1
                    self.models[model]['z'] = 0.25
                    if self.models[model]['y'] >= 0:
                        self.n_line = 1

                # 更新模型位置
                self.set_model_state(
                    model,
                    self.models[model]['x'],
                    self.models[model]['y'],
                    self.models[model]['z']
                )

            # 控制循环频率（0.1秒间隔）
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    controller = ModelController()
    try:
        controller.run()
    except KeyboardInterrupt:
        controller.get_logger().info("程序被用户中断")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
