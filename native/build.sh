#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
python3 "$YZBOT_PROJECT/tools/configure_scene.py"
cmake -S "$YZBOT_WS/src/yzbot/Move_Obs" -B "$YZBOT_WS/src/yzbot/Move_Obs/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$YZBOT_WS/src/yzbot/Move_Obs/build" -j2
export PATH="$YZBOT_PROJECT/native/bin:$PATH"
cd "$YZBOT_WS"
colcon build --base-paths "$YZBOT_WS/src" --build-base "$YZBOT_WS/build" --install-base "$YZBOT_WS/install" --symlink-install --parallel-workers 2 \
  --packages-select linkattacher_msgs ros2_linkattacher mybot_description mybot bot_navigation \
  --cmake-args -DBUILD_TESTING=OFF
echo '原生 Humble 工作区已构建。下一步：./native/start.sh --open'
