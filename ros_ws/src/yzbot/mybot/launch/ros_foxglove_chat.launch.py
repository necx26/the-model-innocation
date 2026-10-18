import launch
import launch_ros
import subprocess
import json
import rclpy
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from std_msgs.msg import String, Int16

class ChatProcessorNode(launch_ros.actions.Node):
    def __init__(self):
        super().__init__(
            package='launch',
            executable='chat_processor',
            name='chat_processor',
            output='screen'
        )
        self.node = None
        self.llama_process = None
        # 跟踪任务状态和剩余数量
        self.current_task_index = 0
        self.tasks = []
        self.current_color = None
        self.current_remaining = 0
        self.current_area = 0
        
    def execute(self, context):
        # 初始化ROS节点
        rclpy.init()
        self.node = rclpy.create_node('chat_processor')
        
        # 创建发布者
        self.llama_command_pub = self.node.create_publisher(String, '/llama_command', 10)
        self.color_pub = self.node.create_publisher(Int16, '/color', 10)
        self.task_pub = self.node.create_publisher(Int16, '/task', 10)
        self.pick_pub = self.node.create_publisher(Int16, '/pick', 10)
        self.stu_pub = self.node.create_publisher(Int16, '/stu', 10)
        self.num_pub = self.node.create_publisher(Int16, '/num', 10)
        self.area_pub = self.node.create_publisher(Int16, '/area', 10)
        
        # 创建订阅者
        self.chat_sub = self.node.create_subscription(
            String,
            '/chat',
            self.chat_callback,
            10
        )
        
        self.llama_command_sub = self.node.create_subscription(
            String,
            '/llama_command',
            self.llama_command_callback,
            10
        )
        
        # 订阅导航状态（修改为独立线程处理）
        self.nav_status_sub = self.node.create_subscription(
            Int16,
            '/nav_status',
            self.nav_status_callback,
            10
        )
        
        # 订阅机械臂状态（修改为独立线程处理）
        self.arm_status_sub = self.node.create_subscription(
            Int16,
            '/arm_status',
            self.arm_status_callback,
            10
        )
        
        self.node.get_logger().info("Chat processor node started")
        
        # 使用多线程spin以确保回调函数能及时处理
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            self.node.get_logger().info("Shutting down chat processor node")
        finally:
            executor.shutdown()
            self.node.destroy_node()
            rclpy.shutdown()
        
        return launch.actions.Shutdown()
    
    def clean_json_response(self, response):
        """清理LLM返回的响应，移除可能的Markdown代码块标记"""
        cleaned = response.strip()
        if cleaned.startswith('```json'):
            cleaned = cleaned[7:].strip()
        if cleaned.endswith('```'):
            cleaned = cleaned[:-3].strip()
        self.node.get_logger().info(f"Cleaned JSON response: {cleaned}")
        return cleaned
    
    def chat_callback(self, msg):
        user_input = msg.data.strip()
        self.node.get_logger().info(f"Received from /chat: {user_input}")
        
        if not user_input or user_input.lower() in ["quit", "exit"]:
            return
            
        # 构建系统提示，加强对输出格式的要求
        system_prompt = '''
            You are a robot pick-and-place assistant.
            Return only pure JSON array with no extra text, no markdown formatting, no code blocks, and no explanations.
            The JSON must be directly parsable.

            Important rules:
            1. Strictly match each number with its corresponding color and target area in the input order
            2. The number (num) must exactly correspond to the quantity specified before its associated color
            3. Preserve the order of items as they appear in the user input

            Example input: "两个蓝色方块到A区，三个红色去B区"
            Example output:[{"color":"blue","num":2,"to":"A"},{"color":"red","num":3,"to":"B"}]
            Example input: "三个红色去C区"
            Example output:[{"color":"red","num":3,"to":"C"}]
            Example input: "5个红色到C区，3个蓝色到A区"
            Example output:[{"color":"red","num":5,"to":"C"},{"color":"blue","num":3,"to":"A"}]
            
            Now extract from user: {{USER_INPUT}}
            '''
        
        # 发送请求到llama-server
        try:
            import requests
            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_input}
            ]
            
            data = {
                "model": "qwen2.5-coder",
                "messages": messages,
                "max_tokens": 512,
                "temperature": 0.0,
                "stream": False,
            }
            
            response = requests.post(
                "http://localhost:8081/v1/chat/completions",
                json=data,
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                assistant_reply = result["choices"][0]["message"]["content"]
                self.node.get_logger().info(f"Raw LLM response: {assistant_reply}")
                
                cleaned_reply = self.clean_json_response(assistant_reply)
                cmd_msg = String()
                cmd_msg.data = cleaned_reply
                self.llama_command_pub.publish(cmd_msg)
                self.node.get_logger().info(f"Published to /llama_command: {cleaned_reply}")
            else:
                self.node.get_logger().error(f"LLM request failed: {response.status_code}, {response.text}")
                
        except Exception as e:
            self.node.get_logger().error(f"Error processing chat: {str(e)}")
    
    def llama_command_callback(self, msg):
        command_data = msg.data.strip()
        self.node.get_logger().info(f"Received from /llama_command: {command_data}")
        
        try:
            cleaned_data = self.clean_json_response(command_data)
            self.tasks = json.loads(cleaned_data)
            self.current_task_index = 0
            self.node.get_logger().info(f"任务队列初始化: {self.tasks}")
            
            # 开始第一个任务
            if self.tasks:
                self.start_next_task()
                
        except json.JSONDecodeError as e:
            self.node.get_logger().error(f"JSON decode error: {str(e)}. Data: {command_data}")
        except Exception as e:
            self.node.get_logger().error(f"Error processing command: {str(e)}")
    
    def start_next_task(self):
        """开始下一个任务或子任务"""
        if self.current_task_index >= len(self.tasks):
            self.node.get_logger().info("所有任务完成")
            # 发布所有等待状态
            self.task_pub.publish(Int16(data=-1))
            self.num_pub.publish(Int16(data=-1))
            self.area_pub.publish(Int16(data=-1))
            self.stu_pub.publish(Int16(data=-1))
            self.pick_pub.publish(Int16(data=-1))
            return
            
        current_task = self.tasks[self.current_task_index]
        self.current_color = current_task.get('color', '').lower()
        self.current_remaining = current_task.get('num', 0)
        target_area = current_task.get('to', '').upper()
        
        # 发布初始任务信息
        task_msg = Int16()
        task_msg.data = 1 if self.current_color == 'red' else 0
        self.task_pub.publish(task_msg)
        
        num_msg = Int16()
        num_msg.data = self.current_remaining
        self.num_pub.publish(num_msg)
        
        area_msg = Int16()
        area_msg.data = 0 if target_area == 'A' else 1 if target_area == 'B' else 2 if target_area == 'C' else -1
        self.area_pub.publish(area_msg)
        self.current_area = area_msg.data
        
        # 发布开始导航状态
        self.stu_pub.publish(Int16(data=1))  # 正在前往目标点
        self.pick_pub.publish(Int16(data=-1))  # 初始化为等待
        
        self.node.get_logger().info(f"开始新任务: {self.current_color}, 剩余: {self.current_remaining}, 区域: {target_area}")
    
    def nav_status_callback(self, msg):
        """处理导航状态更新"""
        status = msg.data
        self.node.get_logger().info(f"收到导航状态: {status}")  # 增加日志用于调试
        if status == 1:  # 到达目标点
            self.stu_pub.publish(Int16(data=2))  # 导航完成
            self.pick_pub.publish(Int16(data=0))  # 准备抓取
            
        elif status == 2:  # 到达仓库
            self.stu_pub.publish(Int16(data=2))  # 导航完成
            self.pick_pub.publish(Int16(data=2))  # 准备放置
    
    def arm_status_callback(self, msg):
        """处理机械臂状态更新"""
        status = msg.data
        self.node.get_logger().info(f"收到机械臂状态: {status}")  # 增加日志用于调试
        if status == 1:  # 抓取完成
            self.pick_pub.publish(Int16(data=1))  # 抓取完成
            # 开始返回仓库
            self.stu_pub.publish(Int16(data=3))  # 返回仓库
            
        elif status == 2:  # 放置完成
            self.current_remaining -= 1
            num_msg = Int16()
            num_msg.data = self.current_remaining
            self.num_pub.publish(num_msg)
            
            # 检查当前任务是否完成
            if self.current_remaining <= 0:
                self.current_task_index += 1
                self.start_next_task()
            else:
                # 继续下一个相同颜色的抓取
                self.stu_pub.publish(Int16(data=1))  # 再次前往目标点
                self.pick_pub.publish(Int16(data=-1))  # 重置抓取状态

def generate_launch_description():
    # 启动llama-server
    llama_server_cmd = ExecuteProcess(
        cmd=[
            'llama-server',
            '-m', '~/llama.cpp/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf',
            '-c', '2048',
            '--threads', '8',
            '--port', '8081'
        ],
        output='screen',
        shell=False
    )
    
    # 启动rosbridge
    rosbridge_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        output='screen',
        shell=False
    )
    
    # 创建聊天处理器节点
    chat_processor_node = ChatProcessorNode()
    
    # 定义退出处理
    exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=llama_server_cmd,
            on_exit=[launch.actions.Shutdown()]
        )
    )
    
    return launch.LaunchDescription([
        llama_server_cmd,
        rosbridge_cmd,
        chat_processor_node,
        exit_handler
    ])
