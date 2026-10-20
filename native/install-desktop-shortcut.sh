#!/usr/bin/env bash
set -eo pipefail
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
desktop=$(xdg-user-dir DESKTOP 2>/dev/null || printf '%s/Desktop' "$HOME")
mkdir -p "$desktop"
python3 - "$project" "$desktop" <<'PY'
from pathlib import Path
import sys
root=Path(sys.argv[1]);desktop=Path(sys.argv[2])
def quoted(p):return '"'+str(p).replace('\\','\\\\').replace('"','\\"').replace('`','\\`').replace('$','\\$').replace('%','%%')+'"'
text='[Desktop Entry]\nVersion=1.0\nType=Application\nName=大模型机器人比赛\nComment=启动机器人比赛与可视化\nExec='+quoted(root/'桌面启动.sh')+'\nPath='+str(root)+'\nIcon='+str(root/'gazebo-robot.svg')+'\nTerminal=true\nStartupNotify=false\nCategories=Development;Science;\n'
for p in (desktop/'大模型机器人比赛.desktop',root/'机器人比赛仿真.desktop'):
 p.write_text(text);p.chmod(0o755)
 print(p)
PY
gio set "$desktop/大模型机器人比赛.desktop" metadata::trusted true 2>/dev/null || true
echo '快捷方式已指向当前工程；若提示未信任，右键选择允许启动。'
