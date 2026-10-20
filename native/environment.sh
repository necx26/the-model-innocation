#!/usr/bin/env bash
# Source this project-local environment in every ROS terminal.
YZBOT_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export YZBOT_PROJECT
export YZBOT_WS="${YZBOT_NATIVE_ROOT:-$YZBOT_PROJECT}/ros_ws"
export YZBOT_LOG="$YZBOT_PROJECT/logs"
export ROS_DOMAIN_ID=${YZBOT_DOMAIN_ID:-91}
if [[ -f /opt/ros/humble/share/rmw_cyclonedds_cpp/package.xml ]]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="${YZBOT_CYCLONEDDS_URI:-file://$YZBOT_WS/cyclonedds.xml}"
else
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  unset CYCLONEDDS_URI
fi
export GAZEBO_MODEL_DATABASE_URI=
export YZBOT_WORLD=offic_room.world
export YZBOT_TASK_MODE=${YZBOT_TASK_MODE:-exam}
export YZBOT_LOCALIZATION=${YZBOT_LOCALIZATION:-slam}
export YZBOT_OFFICIAL_ADAPTER=${YZBOT_OFFICIAL_ADAPTER:-qwen_exam_adapter:solve_task}
export YZBOT_OFFICIAL_TIMEOUT=${YZBOT_OFFICIAL_TIMEOUT:-45}
export YZBOT_PRACTICE_LLM_URL=${YZBOT_PRACTICE_LLM_URL:-http://127.0.0.1:8081}
export QT_X11_NO_MITSHM=1
export LP_NUM_THREADS=${LP_NUM_THREADS:-4}
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
mkdir -p "$YZBOT_LOG"
# Drop other teams' ROS overlays inherited from an interactive .bashrc.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
if [[ -f /usr/share/gazebo/setup.sh ]]; then source /usr/share/gazebo/setup.sh; fi
if [[ -f "$YZBOT_WS/install/local_setup.bash" ]]; then source "$YZBOT_WS/install/local_setup.bash"; fi
if [[ -f "$YZBOT_PROJECT/.runtime/tf2-fix/ros-version.txt" ]] &&
   [[ $(cat "$YZBOT_PROJECT/.runtime/tf2-fix/ros-version.txt") == $(dpkg-query -W -f='${Version}' ros-humble-tf2 2>/dev/null) ]]; then
  export LD_LIBRARY_PATH="$YZBOT_PROJECT/.runtime/tf2-fix${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
export PYTHONPATH="$YZBOT_WS${PYTHONPATH:+:$PYTHONPATH}"
export GAZEBO_PLUGIN_PATH="$YZBOT_WS/install/ros2_linkattacher/lib:$YZBOT_WS/src/yzbot/Move_Obs/build${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
export GAZEBO_MODEL_PATH="$YZBOT_WS/install/mybot_description/share/mybot_description/worlds${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="$YZBOT_WS/install/mybot_description/share/mybot_description/worlds${GAZEBO_RESOURCE_PATH:+:$GAZEBO_RESOURCE_PATH}"
