#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
url=${YZBOT_PRACTICE_LLM_URL:-http://127.0.0.1:8081}
if curl --noproxy '*' -sf "$url/health" >/dev/null; then echo '本地测试模型服务已就绪';exit 0;fi
[[ "$url" == http://127.0.0.1:8081 ]] || { echo "自定义测试模型不可用：$url" >&2;exit 1; }
server=${YZBOT_LLAMA_SERVER:-};model=${YZBOT_QWEN_MODEL:-}
if [[ -z "$server" && -z "$model" ]]; then
  for location in "$HOME/llama.cpp" "$HOME/yzbot_ros2_run/competition/llm" "$root/../../llama.cpp" "$root/models"; do
    if [[ -x "$location/build/bin/llama-server" ]]; then candidate="$location/build/bin/llama-server";else candidate="$location/llama-server";fi
    if [[ -x "$candidate" && -f "$location/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf" ]]; then server="$candidate";model="$location/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf";break;fi
  done
fi
[[ -x "$server" && -f "$model" ]] || { echo '没有找到可选测试 Qwen。按 README 设置 YZBOT_LLAMA_SERVER/YZBOT_QWEN_MODEL，或配置官方模型适配器。包中不包含权重。' >&2;exit 1; }
mkdir -p "$root/logs"
setsid env YZBOT_PROJECT="$root" LD_LIBRARY_PATH="$(dirname -- "$server")${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
 "$server" -m "$model" -c 4096 --threads 4 --host 127.0.0.1 --port 8081 >"$root/logs/host-qwen.log" 2>&1 </dev/null &
echo "$!" >"$root/logs/host-qwen.pid"
for _ in {1..90}; do curl --noproxy '*' -sf "$url/health" >/dev/null && { echo '本地 Qwen 已启动';exit 0; };sleep .5;done
echo '本地 Qwen 未启动成功，查看 logs/host-qwen.log' >&2;exit 1
