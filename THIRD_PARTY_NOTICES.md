# 第三方来源与许可记录

- `ros_ws/src/yzbot/` 的底盘、机械臂、原场景、导航/MoveIt 包来自用户提供的 yzbot 工程；本工程在其基础上修改仿真、导航及比赛流程。保留原源码中的作者、许可证和资源来源标注。部分原包仍标注 TODO 或缺少明确许可，本记录不对这些资产另授许可。
- `ros_ws/src/yzbot/IFRA_LinkAttacher/` 保留其 Apache-2.0 `LICENSE.md` 和原 README。比赛控制器使用此插件在仿真中附着货物。
- Gazebo、ROS 2、MoveIt、Nav2、CycloneDDS、OpenCV、NumPy、PyYAML 等通过系统包在容器构建时安装，各自按其原许可使用；容器镜像和系统二进制不包含于本源码仓库。
- Qwen 权重、llama.cpp 本地推理程序及其共享库未分发，正式模型亦未分发。模型由使用者另行按其许可和比赛要求准备。
- 原工程使用说明另存 `docs/upstream-yzbot-readme.md`，仅作为来源与旧用法记录；当前部署以根目录 README 为准。
