#!/usr/bin/env bash
set -euo pipefail
voice_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 - "$voice_project_dir/assets/voice/试听.html" <<'PY'
import pathlib,sys,webbrowser
webbrowser.open(pathlib.Path(sys.argv[1]).resolve().as_uri())
PY
