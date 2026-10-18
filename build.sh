#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
sudo -v
"$root/start.sh" --daemon-only
dc=(docker -H "unix://$root/docker.sock")
if ! "${dc[@]}" image inspect yzbot-humble:local >/dev/null 2>&1; then
  "${dc[@]}" build --network host -t yzbot-humble:local -f "$root/docker-build/Dockerfile" "$root/docker-build"
fi
"${dc[@]}" run --rm --network host --ipc host \
  --user "$(id -u):$(id -g)" -e HOME=/home/ros -e ROS_DOMAIN_ID=88 \
  -v "$root/runtime-home:/home/ros" \
  -v "$root/ros_ws:/home/ros/dev_ws" -w /home/ros/dev_ws \
  yzbot-humble:local bash -lc 'set -e; source /opt/ros/humble/setup.bash;
    colcon build --symlink-install \
      --packages-select linkattacher_msgs ros2_linkattacher mybot_description mybot bot_navigation \
      --cmake-args -DBUILD_TESTING=OFF;
    cmake -S src/yzbot/Move_Obs -B src/yzbot/Move_Obs/build;
    cmake --build src/yzbot/Move_Obs/build -j2'
echo 'Build complete. Configure a model adapter, then see README for startup.'
