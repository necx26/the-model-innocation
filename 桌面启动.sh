#!/usr/bin/env bash
# Keep a terminal visible so startup errors and log paths can be read.
set +e
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$root" || exit 1
./启动.sh "$@"
status=$?
if (( status != 0 )); then
  echo
  echo "启动未完全就绪。网页诊断页（若服务已启动）：http://127.0.0.1:8765/"
  echo "查看错误：cat logs/startup-error.txt; tail -n 100 logs/controller.log"
  read -r -p '按回车关闭此窗口…' _ || true
fi
exit "$status"
