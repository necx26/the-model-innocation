#!/usr/bin/env bash
set -eo pipefail
xdg-open http://127.0.0.1:8765/fpv >/dev/null 2>&1 &
echo '第一人称：http://127.0.0.1:8765/fpv'
