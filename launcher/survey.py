#!/usr/bin/env python3
import subprocess
import sys
from pathlib import Path
from run import ROOT,ENV,NAME,docker,is_running,native_alias

if Path('/opt/ros/humble/setup.bash').exists():
    native_alias()
    command=['bash','-c','source "$1/native/environment.sh"; exec python3 "$1/tools/collect_search_experience.py" "${@:2}"','survey',str(ROOT),*sys.argv[1:]]
else:
    dc=docker()
    if not is_running(dc):raise SystemExit('请先运行 ./启动.sh')
    command=dc+['exec',NAME,'bash','-c','source /opt/yzbot/native/environment.sh; exec python3 /opt/yzbot/tools/collect_search_experience.py "$@"','survey',*sys.argv[1:]]
raise SystemExit(subprocess.run(command,env=ENV).returncode)
