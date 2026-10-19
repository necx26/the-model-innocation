"""The supplied generator is a CLI: capture one invocation, never invent a task."""
import hashlib
import json
import subprocess
from pathlib import Path

RULES = {
    1: 'x代表红色，y代表蓝色；数量>=3的颜色去A区，另一颜色去C区',
    2: 'x、y中数量>=3的代表红色，另一种代表蓝色；x去B区，y去A区',
    3: '数量多的代表蓝色，少的代表红色；蓝色去B区，红色去C区',
}


def validate_exam_config(value):
    if not isinstance(value, dict):raise ValueError('考试配置必须为对象')
    config = {k:value[k] for k in ('question_mode','rule','first')}
    if config['question_mode'] not in ('competition','practice'):raise ValueError('question_mode 应为 competition/practice')
    if type(config['rule']) is not int or config['rule'] not in RULES:raise ValueError('rule 应为 1/2/3')
    if config['first'] not in ('red','blue'):raise ValueError('first 应为 red/blue')
    return config


def generate_question(binary, config):
    config = validate_exam_config(config)
    binary=Path(binary)
    if not binary.is_file():raise RuntimeError('缺少用户提供的出题程序：'+str(binary))
    command=[str(binary)] + (['11470507'] if config['question_mode']=='competition' else [])
    result=subprocess.run(command,capture_output=True,text=True,encoding='utf-8',timeout=12,check=True)
    text=result.stdout.strip()
    question='\n'.join(line.strip() for line in text.splitlines() if line.strip().startswith(('题目:', '题目：')))
    if not question:raise RuntimeError('出题程序输出中没有“题目:”行，停止而不生成替代题')
    return {'question':question, 'stdout':text, 'context':{
        'rule':config['rule'],'first':config['first'],'mapping_rule':RULES[config['rule']],
        'generator_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
        'generator_mode':config['question_mode']}}


def map_xy(x, y, rule, first):
    if type(x) is not int or type(y) is not int or not 1<=x<=4 or not 1<=y<=4 or x+y!=5:
        raise ValueError('模型解题结果必须是1..4的整数x/y且x+y=5')
    if rule==1:
        task=dict(red=x,blue=y,red_zone='A' if x>=3 else 'C',blue_zone='A' if y>=3 else 'C')
    elif rule==2:
        task=dict(red=max(x,y),blue=min(x,y),red_zone='B' if x>=3 else 'A',blue_zone='B' if x<3 else 'A')
    elif rule==3:
        task=dict(red=min(x,y),blue=max(x,y),red_zone='C',blue_zone='B')
    else:raise ValueError('未知映射规则')
    if first not in ('red','blue'):raise ValueError('需要裁判指定的先后顺序')
    return {**task,'first':first}
