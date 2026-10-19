#!/usr/bin/env bash
set -eo pipefail
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
desktop=$(xdg-user-dir DESKTOP 2>/dev/null || printf '%s/Desktop' "$HOME")
mkdir -p "$desktop"
python3 - "$project" "$desktop" <<'PY'
from pathlib import Path
import sys
root=Path(sys.argv[1]);desktop=Path(sys.argv[2])
def quoted(value):
    return '"'+str(value).replace('\\','\\\\').replace('"','\\"').replace('`','\\`').replace('$','\\$').replace('%','%%')+'"'
for title,script,args,icon in [('机器人比赛启动','start.sh',' --open','applications-engineering'),('机器人比赛关闭','stop.sh','','process-stop')]:
    p=desktop/(title+'.desktop')
    p.write_text('[Desktop Entry]\nType=Application\nName='+title+'\nExec=bash '+quoted(root/'native'/script)+args+'\nPath='+str(root)+'\nIcon='+icon+'\nTerminal=true\nCategories=Education;Science;\n')
    p.chmod(0o755)
    print(p)
PY
for file in "$desktop/机器人比赛启动.desktop" "$desktop/机器人比赛关闭.desktop"; do
  gio set "$file" metadata::trusted true 2>/dev/null || true
done
echo '首次双击若提示未信任，在文件右键菜单选择 Allow Launching。'
