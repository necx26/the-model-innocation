#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
docker_cmd=(docker -H "unix://$root/docker.sock")
curl --noproxy '*' -sf -X POST http://127.0.0.1:8765/api/stop >/dev/null 2>&1 || true
pkill -TERM -f "^$root/competition/llm/llama-server " 2>/dev/null || true
"${docker_cmd[@]}" stop -t 10 yzbot-sim >/dev/null 2>&1 || true
if [[ -f "$root/dockerd.pid" ]]; then
  sudo -n kill "$(cat "$root/dockerd.pid")" 2>/dev/null || true
fi
for _ in {1..30}; do
  "${docker_cmd[@]}" info >/dev/null 2>&1 || break
  sleep 0.5
done
containerd_pid=$(pgrep -f "^containerd --address $root/containerd.sock " || true)
if [[ -n "$containerd_pid" ]]; then
  sudo -n kill $containerd_pid 2>/dev/null || true
fi
