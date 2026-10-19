#!/usr/bin/env python3
"""Reject stale PID files after reboot; do not reuse or signal unrelated processes."""
import os
import sys
from pathlib import Path


def belongs(pid, project):
    try:
        root=Path('/proc')/str(int(pid))
        environ=(root/'environ').read_bytes().split(b'\0')
        return (b'YZBOT_PROJECT='+str(project).encode()) in environ and os.getpgid(int(pid))==int(pid)
    except (ValueError,FileNotFoundError,ProcessLookupError,PermissionError):
        return False


if __name__=='__main__':
    raise SystemExit(0 if belongs(sys.argv[1],sys.argv[2]) else 1)
