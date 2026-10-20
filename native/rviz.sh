#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
if [[ -f "$YZBOT_LOG/rviz.pid" ]] && python3 "$YZBOT_PROJECT/native/process_guard.py" "$(cat "$YZBOT_LOG/rviz.pid")" "$YZBOT_PROJECT"; then
  echo 'RViz 已在运行。'; exit 0
fi
if ! curl --noproxy '*' -sf http://127.0.0.1:8765/api/state >/dev/null; then
  echo '请先在本目录执行 ./启动.sh，等待机器人就绪后再打开 RViz。' >&2
  exit 1
fi
if [[ -z ${XDG_RUNTIME_DIR:-} ]]; then
  export XDG_RUNTIME_DIR="/tmp/yzbot-rviz-$UID"
  mkdir -p "$XDG_RUNTIME_DIR"; chmod 700 "$XDG_RUNTIME_DIR"
fi
export AMENT_PREFIX_PATH="$YZBOT_PROJECT/visualization/rviz_compat:$AMENT_PREFIX_PATH"
export YZBOT_RVIZ_MEDIA="$YZBOT_PROJECT/visualization/rviz_rendering"
export LIBGL_ALWAYS_SOFTWARE=${YZBOT_RVIZ_SOFTWARE_GL:-1}
rm -f "$YZBOT_LOG/rviz-health.json"
setsid "$YZBOT_WS/src/yzbot/Move_Obs/build/competition_rviz" -d "$YZBOT_PROJECT/visualization/competition.rviz" --ros-args -p use_sim_time:=true >"$YZBOT_LOG/rviz.log" 2>&1 </dev/null &
pid=$!; echo "$pid" > "$YZBOT_LOG/rviz.pid"
sleep 1
if ! kill -0 "$pid" 2>/dev/null; then
  echo 'RViz 启动失败，详情见 logs/rviz.log。' >&2
  tail -n 15 "$YZBOT_LOG/rviz.log" >&2;exit 1
fi
ready=0
for _ in {1..60}; do
  if [[ -f "$YZBOT_LOG/rviz-health.json" ]] && python3 - "$YZBOT_LOG/rviz-health.json" <<'PYRVIZ'
import json,sys
try:
    state=json.load(open(sys.argv[1]));sys.exit(0 if state.get('healthy') and len(state.get('displays',[]))>=8 else 1)
except (OSError,ValueError):sys.exit(1)
PYRVIZ
  then ready=1;break;fi
  if ! kill -0 "$pid" 2>/dev/null;then break;fi
  sleep .25
done
if [[ $ready != 1 ]] || grep -q '\[ERROR\]' "$YZBOT_LOG/rviz.log";then
  echo 'RViz 显示尚未全部正常；检查 logs/rviz-health.json 和 logs/rviz.log。' >&2
  exit 1
fi
echo 'RViz 已就绪：模型、地图、TF、激光、路径、轨迹、第一人称显示检查通过。'

