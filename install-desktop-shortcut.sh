#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
python3 - "$root" <<'PY'
import os
import pathlib
import subprocess
import sys
root = pathlib.Path(sys.argv[1])
try:
    desktop = pathlib.Path(subprocess.check_output(['xdg-user-dir', 'DESKTOP'], text=True).strip())
except (FileNotFoundError, subprocess.CalledProcessError):
    desktop = pathlib.Path.home() / 'Desktop'
desktop.mkdir(parents=True, exist_ok=True)
def quoted_exec(path):
    # Desktop Entry Exec quoting, including literal percent signs.
    text = str(path).replace('\\', '\\\\').replace('"', '\\"').replace('`', '\\`').replace('$', '\\$').replace('%', '%%')
    return '"' + text + '"'
entries = [('Gazebo yzbot', root/'desktop-launch.sh', '普通仿真：Gazebo、MoveIt、Nav2'),
           ('yzbot 比赛观察台', root/'competition-start.sh', '比赛练习：需配置可选本地测试模型')]
for name, script, description in entries:
    path = desktop / (name + '.desktop')
    icon = str(root/'gazebo-robot.svg').replace('\\', '\\\\')
    path.write_text('[Desktop Entry]\nVersion=1.0\nType=Application\nName='+name+
                    '\nComment='+description+'\nExec='+quoted_exec(script)+
                    '\nIcon='+icon+'\nTerminal=false\nCategories=Development;Science;\n')
    path.chmod(0o755)
    try:
        subprocess.run(['gio', 'set', str(path), 'metadata::trusted', 'true'], check=False, capture_output=True)
    except FileNotFoundError:
        pass
    print(path)
PY
