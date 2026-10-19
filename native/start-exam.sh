#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
python3 - "$@" <<'PY'
import argparse,json,urllib.request
p=argparse.ArgumentParser();p.add_argument('--rule',required=True,type=int,choices=[1,2,3]);p.add_argument('--first',required=True,choices=['red','blue']);p.add_argument('--practice',action='store_true');a=p.parse_args()
op=urllib.request.build_opener(urllib.request.ProxyHandler({}))
def post(path,data=None):
    req=urllib.request.Request('http://127.0.0.1:8765'+path,data=json.dumps(data).encode() if data is not None else b'',headers={'Content-Type':'application/json'})
    return json.load(op.open(req,timeout=5))
print(post('/api/exam-config',dict(rule=a.rule,first=a.first,question_mode='practice' if a.practice else 'competition')))
r=post('/api/start');print(r)
if not r.get('started'):raise SystemExit('没有开始；检查观察台状态，已搬过货物则先复位')
PY
