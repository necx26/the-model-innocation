#!/usr/bin/env python3
import argparse
import json
import time
import urllib.request
p=argparse.ArgumentParser();p.add_argument('action',choices=['start','reset']);p.add_argument('--rule',type=int,choices=[1,2,3],default=1);p.add_argument('--first',choices=['red','blue'],default='red');p.add_argument('--practice',action='store_true');a=p.parse_args()
op=urllib.request.build_opener(urllib.request.ProxyHandler({}))
def post(path,value=None):
    req=urllib.request.Request('http://127.0.0.1:8765'+path,data=json.dumps(value).encode() if value is not None else b'',headers={'Content-Type':'application/json'})
    try:
        with op.open(req,timeout=5) as r:return json.load(r)
    except urllib.error.HTTPError as e:raise SystemExit(e.read().decode())
if a.action=='start':
    post('/api/exam-config',dict(rule=a.rule,first=a.first,question_mode='practice' if a.practice else 'competition'))
    if not post('/api/start').get('started'):raise SystemExit('未能开始，请检查页面状态并先重置。')
    print('考试已开始：http://127.0.0.1:8765/')
else:
    path='/api/reset'
    if not post(path).get('resetting'):raise SystemExit('未能重置；任务正在运行时请先在页面停止作业。')
    for _ in range(240):
        with op.open('http://127.0.0.1:8765/api/state',timeout=5) as r:s=json.load(r)
        if s['phase']=='重置失败':raise SystemExit(s['status'])
        if not s.get('busy') and s['phase']=='就绪':print('场景与导航重置完成');break
        time.sleep(.5)
    else:raise SystemExit('等待重置超时，请看页面状态。')
