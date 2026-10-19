#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
curl --noproxy '*' -sf -X POST http://127.0.0.1:8765/api/stop >/dev/null || true
python3 - "$YZBOT_LOG" <<'PY'
import os,signal,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(os.environ['YZBOT_PROJECT'])/'native'))
from process_guard import belongs
root=Path(sys.argv[1]);groups=[]
project=os.environ['YZBOT_PROJECT'].encode()
own={os.getpid()};parent=os.getppid()
while parent>1:
    own.add(parent)
    try:parent=int((Path('/proc')/str(parent)/'stat').read_text().split()[3])
    except (FileNotFoundError,ValueError):break
for name in ['controller','foxglove','navigation','gazebo','qwen','viewer']:
    f=root/(name+'.pid')
    if not f.exists():continue
    try:
        pid=int(f.read_text());cmd=Path('/proc')/str(pid)/'cmdline'
        if not cmd.exists() or not belongs(pid,os.environ['YZBOT_PROJECT']):continue
        # Only stop this project's stored process group, never blanket-pkill ROS.
        group=os.getpgid(pid)
        if group!=pid:continue
        os.killpg(group,signal.SIGTERM);groups.append(group)
    except (ProcessLookupError,ValueError,PermissionError):pass
# Gazebo's launcher creates separate process groups for gzserver/gzclient.
# Match this project's environment rather than killing unrelated Gazebo sessions.
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit() or int(entry.name) in own:continue
    try:
        if b'YZBOT_PROJECT='+project+b'\0' not in (entry/'environ').read_bytes():continue
        command=(entry/'cmdline').read_bytes().replace(b'\0',b' ')
        executable=(entry/'comm').read_text().strip()
        if executable in ('bash','sh','zsh'):continue
        if executable.startswith('python') and not any(x in command for x in (b'competition_node',b'/opt/ros/',b'wait_ready.py',b'visualization_bridge.py')):continue
        os.kill(int(entry.name),signal.SIGTERM)
    except (FileNotFoundError,ProcessLookupError,PermissionError):pass
for _ in range(20):
    alive=[]
    for group in groups:
        try:os.killpg(group,0);alive.append(group)
        except ProcessLookupError:pass
    if not alive:break
    groups=alive;time.sleep(.2)
for group in groups:
    try:os.killpg(group,signal.SIGKILL)
    except ProcessLookupError:pass
print('本工程进程已停止；没有删除源码、模型或日志。')
PY
