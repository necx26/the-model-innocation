#!/usr/bin/env bash
set -euo pipefail

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$root/logs" "$root/runtime-home"
exec 9>"$root/start.lock"
flock -n 9 || exit 0

docker_cmd=(docker -H "unix://$root/docker.sock")
if ! "${docker_cmd[@]}" info >/dev/null 2>&1; then
  if ! pgrep -f "containerd --address $root/containerd.sock" >/dev/null; then
    nohup sudo -n containerd \
      --address "$root/containerd.sock" \
      --root "$root/containerd-root" \
      --state "$root/containerd-state" \
      >"$root/logs/containerd.log" 2>&1 </dev/null &
  fi
  for _ in {1..40}; do
    sudo -n ctr --address "$root/containerd.sock" version >/dev/null 2>&1 && break
    sleep 0.5
  done
  if ! pgrep -f "dockerd --data-root=$root/docker-data" >/dev/null; then
    nohup sudo -n dockerd \
      --data-root="$root/docker-data" \
      --exec-root="$root/docker-exec" \
      --pidfile="$root/dockerd.pid" \
      --group="$(id -gn)" \
      --host="unix://$root/docker.sock" \
      --containerd="$root/containerd.sock" \
      --bridge=none --iptables=false --ip6tables=false --userland-proxy=false \
      >"$root/logs/dockerd.log" 2>&1 </dev/null &
  fi
  for _ in {1..90}; do
    "${docker_cmd[@]}" info >/dev/null 2>&1 && break
    sleep 1
  done
fi
"${docker_cmd[@]}" info >/dev/null

if [[ "${1:-}" == --daemon-only ]]; then exit 0; fi

if ! "${docker_cmd[@]}" image inspect yzbot-humble:local >/dev/null 2>&1; then
  echo "Missing yzbot-humble:local image. See $root/logs/build-image.log" >&2
  exit 1
fi

if ! "${docker_cmd[@]}" inspect -f '{{.State.Running}}' yzbot-sim 2>/dev/null | grep -qx true; then
  "${docker_cmd[@]}" rm -f yzbot-sim >/dev/null 2>&1 || true
  display=${DISPLAY:-:0}
  xauthority=${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}
  "${docker_cmd[@]}" run -d --rm --name yzbot-sim \
    --network host --ipc host --user "$(id -u):$(id -g)" \
    -e HOME=/home/ros -e DISPLAY="$display" -e XAUTHORITY=/home/ros/.Xauthority \
    -e ROS_DOMAIN_ID=88 -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e LP_NUM_THREADS=4 -e YZBOT_WORLD="${YZBOT_WORLD:-3.world}" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///home/ros/dev_ws/cyclonedds.xml \
    -e GAZEBO_MODEL_DATABASE_URI= \
    -v "$root/runtime-home:/home/ros" \
    -v "$xauthority:/home/ros/.Xauthority:ro" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$root/ros_ws:/home/ros/dev_ws" \
    -v "$root/logs:/home/ros/logs" \
    --workdir /home/ros/dev_ws yzbot-humble:local sleep infinity >/dev/null
fi

launch() {
  local name=$1 package=$2 file=$3
  if "${docker_cmd[@]}" exec yzbot-sim pgrep -f "[r]os2 launch $package $file" >/dev/null 2>&1; then
    return
  fi
  "${docker_cmd[@]}" exec -d yzbot-sim bash -lc \
    "source /opt/ros/humble/setup.bash; source /home/ros/dev_ws/install/setup.bash; exec ros2 launch $package $file > /home/ros/logs/$name.log 2>&1"
}

launch gazebo mybot gazebo_world.launch.py
for _ in {1..90}; do
  if "${docker_cmd[@]}" exec yzbot-sim bash -lc \
    'source /opt/ros/humble/setup.bash; source /home/ros/dev_ws/install/setup.bash; ros2 topic list' 2>/dev/null | grep -qx '/clock'; then
    break
  fi
  sleep 1
done
if [[ "${YZBOT_COMPETITION:-0}" == 1 ]]; then
  launch competition mybot competition.launch.py
  "${docker_cmd[@]}" exec yzbot-sim bash -lc \
    'source /opt/ros/humble/setup.bash; timeout 35 bash -c '\''until ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active"; do sleep .5; done'\'''
  exit 0
fi
launch moveit mybot my_moveit_rviz.launch.py
nav_already_running=false
if "${docker_cmd[@]}" exec yzbot-sim pgrep -f '[r]os2 launch bot_navigation nav_bringup_gazebo.launch.py' >/dev/null 2>&1; then
  nav_already_running=true
fi
launch nav2 bot_navigation nav_bringup_gazebo.launch.py
if [[ "$nav_already_running" == false ]]; then
  "${docker_cmd[@]}" exec yzbot-sim bash -lc \
    'source /opt/ros/humble/setup.bash; source /home/ros/dev_ws/install/setup.bash; python3 /home/ros/dev_ws/initial_pose.py' \
    >"$root/logs/initial-pose.log" 2>&1
fi
command -v notify-send >/dev/null && notify-send 'Gazebo 机器人仿真' '仿真正在启动。日志在 yzbot_ros2_run/logs。' || true
