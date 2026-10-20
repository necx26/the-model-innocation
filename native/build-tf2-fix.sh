#!/usr/bin/env bash
set -eo pipefail
root=${YZBOT_PROJECT:?先加载 native/environment.sh}
version=$(dpkg-query -W -f='${Version}' ros-humble-tf2)
if [[ $version != 0.25.23-* ]]; then
  echo "tf2 $version：未应用仅针对 0.25.23 的项目修复。"
  exit 0
fi
cmake -S "$root/runtime_fixes/tf2" -B "$root/.runtime/tf2-fix" -DCMAKE_BUILD_TYPE=Release
cmake --build "$root/.runtime/tf2-fix" -j2
LD_LIBRARY_PATH="$root/.runtime/tf2-fix${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" timeout 5 "$root/.runtime/tf2-fix/lock_order_regression"
printf '%s' "$version" > "$root/.runtime/tf2-fix/ros-version.txt"
