#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
python3 "$YZBOT_PROJECT/native/navigation_process.py"
setsid ros2 launch mybot competition.launch.py >"$YZBOT_LOG/navigation.log" 2>&1 </dev/null &
echo "$!" >"$YZBOT_LOG/navigation.pid"
python3 "$YZBOT_PROJECT/native/wait_ready.py" nav --timeout 70
