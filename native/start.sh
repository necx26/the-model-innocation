#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
[[ -f "$YZBOT_WS/install/local_setup.bash" ]] || { echo '先执行 ./native/build.sh' >&2;exit 1; }
if current=$(curl --noproxy '*' -sf http://127.0.0.1:8765/api/state); then
  if python3 -c 'import json,sys; a=json.load(sys.stdin); sys.exit(0 if a.get("healthy") and a.get("phase")!="重置失败" else 1)' <<< "$current"; then
    echo '工程已运行且机器人就绪：http://127.0.0.1:8765'
    if [[ ${YZBOT_ENABLE_RVIZ:-1} == 1 ]]; then "$YZBOT_PROJECT/native/rviz.sh"; fi
    if [[ ${1:-} == --open ]]; then "$YZBOT_PROJECT/native/visualize.sh"; fi
    exit 0
  fi
  echo '检测到未就绪的旧进程，先停止本工程再重新启动。'
  "$YZBOT_PROJECT/native/stop.sh"
fi
if [[ "$YZBOT_OFFICIAL_ADAPTER" == qwen_exam_adapter:solve_task ]]; then "$YZBOT_PROJECT/native/start-model.sh"; fi
launch() {
  local name=$1;shift
  if [[ -f "$YZBOT_LOG/$name.pid" ]] && python3 "$YZBOT_PROJECT/native/process_guard.py" "$(cat "$YZBOT_LOG/$name.pid")" "$YZBOT_PROJECT"; then
    echo "复用运行中的 $name";return
  fi
  setsid "$@" >"$YZBOT_LOG/$name.log" 2>&1 </dev/null &
  echo "$!" >"$YZBOT_LOG/$name.pid"
}
echo '[1/4] 启动 Gazebo 与原小车、机械臂、传感器'
launch gazebo ros2 launch mybot gazebo_world.launch.py
python3 "$YZBOT_PROJECT/native/wait_ready.py" gazebo
python3 "$YZBOT_PROJECT/native/reset_robot.py"
echo '[2/4] 启动 Nav2 与 MoveIt'
launch navigation ros2 launch mybot competition.launch.py
python3 "$YZBOT_PROJECT/native/wait_ready.py" nav
echo '[3/4] 启动考试桥与第一人称观察页'
launch controller python3 -u "$YZBOT_WS/competition_node.py"
for _ in {1..60}; do curl --noproxy '*' -sf http://127.0.0.1:8765/api/state >/dev/null && break; sleep .5; done
curl --noproxy '*' -sf http://127.0.0.1:8765/api/state >/dev/null
curl --noproxy '*' -sf -X POST 'http://127.0.0.1:8765/api/reset?mode=fixed' >/dev/null
python3 - <<'PY'
import json,time,urllib.request
op=urllib.request.build_opener(urllib.request.ProxyHandler({}))
for _ in range(180):
    state=json.load(op.open('http://127.0.0.1:8765/api/state',timeout=3))
    if state['phase']=='重置失败':raise SystemExit(state['status'])
    if state.get('layout_mode')=='fixed_distributed' and not state.get('layout_randomized') and state['phase']=='就绪':break
    time.sleep(.5)
else:raise SystemExit('场景初始化超时')
PY
echo '[4/4] 本地可视化已就绪'
if [[ ${YZBOT_ENABLE_RVIZ:-1} == 1 ]]; then "$YZBOT_PROJECT/native/rviz.sh"; fi
# Optional compatibility bridge for existing external Foxglove clients.
if [[ ${YZBOT_ENABLE_FOXGLOVE:-0} == 1 ]]; then
  launch foxglove python3 -u "$YZBOT_PROJECT/native/visualization_bridge.py"
fi
echo '场景已就绪。确认规则与顺序后，按开始考试才取题并开始300秒计时。'
echo '面板 http://127.0.0.1:8765 | 第一人称 http://127.0.0.1:8765/fpv'
if [[ ${1:-} == --open ]]; then "$YZBOT_PROJECT/native/visualize.sh"; fi
