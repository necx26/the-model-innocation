#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
url='http://127.0.0.1:8765/'
if ! curl --noproxy '*' -sf "$url" >/dev/null; then
  echo '仿真尚未启动。先执行 ./native/start.sh --open' >&2
  exit 1
fi
xdg-open "$url" >/dev/null 2>&1 &
echo "本地可视化：$url（无需 Foxglove、账号或外网）"
