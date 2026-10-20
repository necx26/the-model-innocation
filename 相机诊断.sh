#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source native/environment.sh
python3 native/camera_probe.py "$@"
