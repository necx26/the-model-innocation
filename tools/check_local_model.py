#!/usr/bin/env python3
"""Start this system's optional Qwen on an unused port; solve a real exam question."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import urllib.request
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'ros_ws'))
from exam_bridge import generate_question
from qwen_exam_adapter import solve_task

log=Path(os.environ.get('YZBOT_LOG',root/'logs'))/'independent-qwen-check'
log.mkdir(parents=True,exist_ok=True)
server=os.environ.get('YZBOT_LLAMA_SERVER',str(Path.home()/'llama.cpp/build/bin/llama-server'))
model=os.environ.get('YZBOT_QWEN_MODEL',str(Path.home()/'llama.cpp/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf'))
with socket.socket() as sock:
    sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
os.environ['YZBOT_LOG']=str(log)
os.environ['YZBOT_PRACTICE_LLM_URL']=f'http://127.0.0.1:{port}'
op=urllib.request.build_opener(urllib.request.ProxyHandler({}))
with (log/'llama-server.log').open('w') as output:
    process=subprocess.Popen([server,'-m',model,'-c','4096','--threads','4','--host','127.0.0.1','--port',str(port)],stdout=output,stderr=subprocess.STDOUT)
    try:
        for _ in range(90):
            if process.poll() is not None:raise RuntimeError('Qwen 已退出，检查 '+str(log/'llama-server.log'))
            try:
                health=json.load(op.open(f'http://127.0.0.1:{port}/health',timeout=1))
                if health.get('status')=='ok':break
            except Exception:pass
            time.sleep(.5)
        else:raise TimeoutError('Qwen 健康检查超时')
        question=generate_question(root/'competition/platform/TMSCQtest_x86_x64.bin',dict(question_mode='competition',rule=1,first='red'))
        task=solve_task(question=question['question'],context=question['context'],config={})
        proof=dict(server=server,model=model,port=port,health=health,task=task)
        (log/'result.json').write_text(json.dumps(proof,ensure_ascii=False,indent=2))
        print(json.dumps(proof,ensure_ascii=False))
    finally:
        process.terminate()
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()
