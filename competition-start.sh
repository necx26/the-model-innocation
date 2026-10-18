#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$root/logs"
mode=${YZBOT_TASK_MODE:-practice}
[[ "$mode" == practice || "$mode" == official ]] || { echo 'YZBOT_TASK_MODE must be practice or official' >&2; exit 1; }
open_panel() {
  if command -v google-chrome >/dev/null; then
    nohup google-chrome --new-window http://127.0.0.1:8765 >/dev/null 2>&1 </dev/null &
  elif command -v xdg-open >/dev/null; then
    nohup xdg-open http://127.0.0.1:8765 >/dev/null 2>&1 </dev/null &
  fi
}
if current=$(curl --noproxy '*' -sf http://127.0.0.1:8765/api/state); then
  active_mode=$(python3 -c 'import json,sys; print(json.load(sys.stdin).get("task_mode","practice"))' <<< "$current")
  [[ "$active_mode" == "$mode" ]] || { echo "Already running in $active_mode mode. Run ./stop.sh before switching to $mode." >&2; exit 1; }
  open_panel
  exit 0
fi
if [[ "$mode" == practice ]]; then
  practice_url=${YZBOT_PRACTICE_LLM_URL:-http://127.0.0.1:8081}
  if ! curl --noproxy '*' -sf "$practice_url/health" >/dev/null; then
    if [[ "$practice_url" != http://127.0.0.1:8081 || ! -x "$root/competition/llm/llama-server" || ! -f "$root/competition/llm/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf" ]]; then
      echo 'Practice model unavailable. Qwen/llama-server are not distributed. See README section 5; configure a local test service or use YZBOT_TASK_MODE=official.' >&2
      exit 1
    fi
    nohup env LD_LIBRARY_PATH="$root/competition/llm" \
      "$root/competition/llm/llama-server" \
      -m "$root/competition/llm/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf" \
      -c 2048 --threads 4 --port 8081 --host 127.0.0.1 \
      >"$root/logs/llm.log" 2>&1 </dev/null &
    for _ in {1..60}; do
      curl --noproxy '*' -sf "$practice_url/health" >/dev/null && break
      sleep .5
    done
    curl --noproxy '*' -sf "$practice_url/health" >/dev/null
  fi
fi
YZBOT_WORLD=competition.world YZBOT_COMPETITION=1 "$root/start.sh"
docker -H "unix://$root/docker.sock" exec -d \
  -e YZBOT_TASK_MODE="$mode" \
  -e YZBOT_PRACTICE_LLM_URL="${YZBOT_PRACTICE_LLM_URL:-http://127.0.0.1:8081}" \
  -e YZBOT_OFFICIAL_ADAPTER="${YZBOT_OFFICIAL_ADAPTER:-official_model_adapter:solve_task}" \
  -e YZBOT_OFFICIAL_TIMEOUT="${YZBOT_OFFICIAL_TIMEOUT:-45}" \
  -e YZBOT_OFFICIAL_CONFIG="${YZBOT_OFFICIAL_CONFIG:-}" \
  yzbot-sim bash -lc \
  'source /opt/ros/humble/setup.bash; source /home/ros/dev_ws/install/setup.bash; exec python3 -u /home/ros/dev_ws/competition_node.py > /home/ros/logs/competition-manager.log 2>&1'
for _ in {1..60}; do
  curl --noproxy '*' -sf http://127.0.0.1:8765/api/state >/dev/null && break
  sleep .5
done
curl --noproxy '*' -sf http://127.0.0.1:8765/api/state >/dev/null
open_panel
printf 'Dashboard: http://127.0.0.1:8765 (mode=%s)\n' "$mode"
