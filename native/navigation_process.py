#!/usr/bin/env python3
"""Stop only a verified project-owned navigation process group."""
import json
import os
import signal
import time
from pathlib import Path
from process_guard import belongs


def live_members(group):
    members=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:
            pid=int(entry.name)
            if os.getpgid(pid)!=group:continue
            if (entry/'stat').read_text().rsplit(')',1)[1].split()[0]=='Z':continue
            members.append(pid)
        except (OSError,ValueError):continue
    return members


def terminate_group(pid,project,timeout=6):
    if pid==os.getpgrp() or not belongs(pid,project):
        raise RuntimeError('导航 PID 不属于本工程独立进程组，拒绝发送停止信号')
    os.killpg(pid,signal.SIGTERM)
    deadline=time.monotonic()+timeout
    while live_members(pid) and time.monotonic()<deadline:time.sleep(.05)
    members=live_members(pid)
    if members:
        expected=b'YZBOT_PROJECT='+str(project).encode()
        for member in members:
            try:environment=(Path('/proc')/str(member)/'environ').read_bytes().split(b'\0')
            except FileNotFoundError:continue
            if expected not in environment:raise RuntimeError('进程组出现非本工程成员，拒绝强制终止')
        os.killpg(pid,signal.SIGKILL)
        deadline=time.monotonic()+2
        while live_members(pid) and time.monotonic()<deadline:time.sleep(.05)
        if live_members(pid):raise RuntimeError('旧导航进程组未完全退出')
    return bool(members)


def main():
    project=Path(os.environ['YZBOT_PROJECT']);log=project/'logs'
    pid=int((log/'navigation.pid').read_text())
    forced=terminate_group(pid,project)
    directory=log/'navigation-resets'/str(time.time_ns());directory.mkdir(parents=True)
    if (log/'navigation.log').exists():(log/'navigation.log').rename(directory/'navigation.log')
    (log/'navigation-reset-latest.json').write_text(json.dumps(
        {'stopped_group':pid,'forced_stop':forced,'archived_log':str(directory.relative_to(project))},indent=2))
    print('本工程旧导航与 SLAM 已停止；Gazebo、相机和控制器继续运行。',flush=True)


if __name__=='__main__':main()
