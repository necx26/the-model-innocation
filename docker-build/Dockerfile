FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN if [ -f /etc/apt/sources.list ]; then \
      sed -i 's#http://archive.ubuntu.com/ubuntu#https://mirrors.cloud.tencent.com/ubuntu#g; s#http://security.ubuntu.com/ubuntu#https://mirrors.cloud.tencent.com/ubuntu#g' /etc/apt/sources.list; \
    fi \
    && cp /usr/share/ros-apt-source/ros2.sources /etc/apt/sources.list.d/ros2.local.sources \
    && rm /etc/apt/sources.list.d/ros2.sources \
    && sed -i 's#Types: deb deb-src#Types: deb#; s#http://packages.ros.org/ros2/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu#' /etc/apt/sources.list.d/ros2.local.sources \
    && apt-get -o Acquire::ForceIPv4=true update \
    && apt-get -o Acquire::ForceIPv4=true install -y --no-install-recommends \
    gazebo \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-moveit \
    ros-humble-moveit-setup-assistant \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-trajectory-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    libgazebo-dev \
    libboost-system-dev \
    libprotobuf-dev \
    protobuf-compiler \
    build-essential \
    cmake \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN if ! getent group 1000 >/dev/null; then groupadd -g 1000 ros; fi \
    && if ! getent passwd 1000 >/dev/null; then useradd -m -u 1000 -g 1000 -s /bin/bash ros; fi \
    && mkdir -p /home/ros \
    && chown 1000:1000 /home/ros

ENV HOME=/home/ros
WORKDIR /home/ros/dev_ws
