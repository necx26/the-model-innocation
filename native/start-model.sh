#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
if curl --noproxy '*' -sf "$YZBOT_PRACTICE_LLM_URL/health" >/dev/null; then echo 'Qwen 服务已就绪'; exit 0; fi
if [[ "$YZBOT_PRACTICE_LLM_URL" != http://127.0.0.1:8081 ]]; then echo '自定义测试模型服务未就绪' >&2;exit 1; fi
model=${YZBOT_QWEN_MODEL:-$HOME/llama.cpp/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf}
server=${YZBOT_LLAMA_SERVER:-$HOME/llama.cpp/build/bin/llama-server}
[[ -f "$model" && -x "$server" ]] || { echo '缺少可选本地 Qwen；设置 YZBOT_QWEN_MODEL 和 YZBOT_LLAMA_SERVER。源码包不含权重。' >&2;exit 1; }
setsid "$server" -m "$model" -c 4096 --threads 4 --host 127.0.0.1 --port 8081 \
  >"$YZBOT_LOG/qwen.log" 2>&1 </dev/null &
echo "$!" >"$YZBOT_LOG/qwen.pid"
for _ in {1..90}; do curl --noproxy '*' -sf "$YZBOT_PRACTICE_LLM_URL/health" >/dev/null && exit 0; sleep .5; done
echo 'Qwen 启动超时，查看 logs/qwen.log' >&2;exit 1
