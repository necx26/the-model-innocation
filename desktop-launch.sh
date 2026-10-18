#!/usr/bin/env bash
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$root/logs"
if ! "$root/start.sh" >>"$root/logs/desktop.log" 2>&1; then
  zenity --error --title='Gazebo 机器人仿真' \
    --text="启动失败，请查看 $root/logs/desktop.log" 2>/dev/null || true
  exit 1
fi
