#!/usr/bin/env python3
"""Update the single project README and machine-readable test index."""
import hashlib,json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def replace_block(path,marker,text):
    s=path.read_text();a='<!-- '+marker+' -->';b='<!-- /'+marker+' -->'
    if a not in s or b not in s:raise ValueError('README validation marker missing')
    path.write_text(re.sub(re.escape(a)+r'.*?'+re.escape(b),lambda _:a+'\n\n'+text+'\n\n'+b,s,flags=re.S))
def update():
    paths=sorted((ROOT/'verification/mission_runs').glob('*/round-*.json'),key=lambda p:p.stat().st_mtime)
    if not paths:return
    records=[];rows=[]
    for p in paths:
        r=json.loads(p.read_text());s=r['final_state'];t=r['task'];good=sum(r['physical_checks'])
        source='原题回归' if 'replay' in r.get('question_source','') else '实时出题'
        records.append(dict(run=p.parent.name,passed=r['passed'],elapsed=r['elapsed'],remaining=r['remaining'],task=t,physical_checks=r['physical_checks'],source=source))
        task=f"红{t['red']}→{t['red_zone']} / 蓝{t['blue']}→{t['blue_zone']}，{'红' if t['first']=='red' else '蓝'}先"
        rows.append(f"| {source} | {task} | {good}/5 | {s.get('grasp_attempts','—')} | {r['elapsed']:.1f} | {r['remaining']:.1f} | {'通过' if r['passed'] else '未通过'} |")
    (ROOT/'verification/summary.json').write_text(json.dumps(records,ensure_ascii=False,indent=2))
    r=json.loads(paths[-1].read_text());s=r['final_state']
    match=all((ROOT/n).exists() and hashlib.sha256((ROOT/n).read_bytes()).hexdigest()==v for n,v in s.get('runtime_source_files_sha256',{}).items())
    match=match and hashlib.sha256((ROOT/'ros_ws/competition_node.py').read_bytes()).hexdigest()==s.get('runtime_source_sha256')
    note='当前执行源码已完成本轮复验。' if match else '该轮之后有源码更新，需以更新后的完整复验为准。'
    table='\n'.join(['| 来源 | 任务 | 最终合格 | 抓取尝试 | 用时秒 | 余量秒 | 结果 |','| --- | --- | ---: | ---: | ---: | ---: | --- |',*rows[-6:]])
    replace_block(ROOT/'README.md','LATEST_VALIDATION',table+'\n\n'+note+' 测试记录位于 `verification/mission_runs/`，历史统计位于 `verification/history-summary.json`。')
    return str(ROOT/'README.md')
if __name__=='__main__':print(update())
