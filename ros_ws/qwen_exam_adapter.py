"""Local Qwen + auditable arithmetic tool. No replacement answer on LLM failure."""
import json
import os
import urllib.request
from pathlib import Path
from question_math import demand_tool
from exam_bridge import map_xy


def solve_task(*, question, context, config):
    tool=demand_tool(question)
    schema={'type':'object','properties':{
        'x':{'type':'integer','minimum':1,'maximum':4},
        'y':{'type':'integer','minimum':1,'maximum':4}},'required':['x','y'],'additionalProperties':False}
    # The LLM is explicitly provided an arithmetic-tool observation. Its answer
    # must agree; a wrong or unavailable model is never silently bypassed.
    messages=[{'role':'system','content':'You solve the original question with an arithmetic tool. Return the verified tool x/y exactly as JSON. Never change the tool result.'},
              {'role':'user','content':'Original question: '+question+'\nVerified demand/arithmetic tool: '+json.dumps(tool,ensure_ascii=False)}]
    base=os.environ.get('YZBOT_PRACTICE_LLM_URL','http://127.0.0.1:8081').rstrip('/')
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    attempts=[]
    for _ in range(2):
        body=dict(messages=messages,temperature=0,max_tokens=100,response_format={'type':'json_object','schema':schema})
        req=urllib.request.Request(base+'/v1/chat/completions',data=json.dumps(body).encode(),headers={'Content-Type':'application/json'})
        with opener.open(req,timeout=20) as response:raw=json.load(response)
        answer=json.loads(raw['choices'][0]['message']['content']);attempts.append(answer)
        if answer=={'x':tool['x'],'y':tool['y']}:break
        messages.extend([{'role':'assistant','content':json.dumps(answer)},
                         {'role':'user','content':'The answer disagrees with the arithmetic tool. Copy x and y from the tool exactly.'}])
    else:raise RuntimeError('Qwen结果与题意算术工具不一致，停止本轮；不会替换为工具答案')
    task=map_xy(answer['x'],answer['y'],int(context['rule']),context['first'])
    log=Path(os.environ.get('YZBOT_LOG') or Path(__file__).resolve().parent/'logs')
    log.mkdir(parents=True,exist_ok=True)
    (log/'exam-model-result.json').write_text(json.dumps(dict(question=question,tool=tool,model_attempts=attempts,task=task,
        provider='local_Qwen_with_demand_arithmetic_tool',generator=context.get('generator_sha256')),ensure_ascii=False,indent=2))
    return task
