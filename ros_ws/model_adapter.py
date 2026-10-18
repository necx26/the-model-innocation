"""Project task contract; official protocol belongs in the configured adapter."""
import importlib
import json
import math
import os
import threading
from pathlib import Path


def validate_task(value):
    fields = {'red', 'blue', 'first', 'red_zone', 'blue_zone'}
    if not isinstance(value, dict) or set(value) != fields:
        raise ValueError('模型任务必须且只能包含 red, blue, first, red_zone, blue_zone')
    if any(type(value[c]) is not int or not 0 <= value[c] <= 5 for c in ('red', 'blue')):
        raise ValueError('red/blue 必须为 0 到 5 的整数')
    if value['red'] + value['blue'] != 5:
        raise ValueError('当前仿真执行器仅支持总数 5；其他赛题需扩展执行器')
    if value['first'] not in ('red', 'blue'):
        raise ValueError('first 必须为 red 或 blue')
    if any(value[c] not in ('A', 'B', 'C') for c in ('red_zone', 'blue_zone')):
        raise ValueError('放置区必须为 A/B/C')
    return dict(value)


def invoke_official(question, context):
    if not isinstance(question, str) or not question.strip():
        raise ValueError('尚未收到官方题目')
    if not isinstance(context, dict):
        raise ValueError('context 必须为 JSON 对象')
    spec = os.environ.get('YZBOT_OFFICIAL_ADAPTER', 'official_model_adapter:solve_task')
    module, separator, function = spec.partition(':')
    if not separator or not module or not function:
        raise ValueError('YZBOT_OFFICIAL_ADAPTER 应为 module:function')
    config_path = os.environ.get('YZBOT_OFFICIAL_CONFIG', '')
    config = json.loads(Path(config_path).read_text()) if config_path else {}
    if not isinstance(config, dict):
        raise ValueError('官方适配配置必须为 JSON 对象')
    # No local generator, answer oracle, or Qwen fallback exists on this path.
    adapter = getattr(importlib.import_module(module), function)
    timeout = float(os.environ.get('YZBOT_OFFICIAL_TIMEOUT', '45'))
    if not math.isfinite(timeout) or not 0 < timeout <= 240:
        raise ValueError('YZBOT_OFFICIAL_TIMEOUT 必须大于 0 且不超过 240 秒')
    done, result = threading.Event(), {}
    def infer():
        try:
            result['task'] = adapter(question=question, context=dict(context), config=config)
        except Exception as error:
            result['error'] = error
        finally:
            done.set()
    # The adapter must perform inference only, never issue robot commands.
    # The deadline stops this mission; SDK-level timeouts must also release IO.
    threading.Thread(target=infer, daemon=True).start()
    if not done.wait(timeout):
        raise TimeoutError(f'官方模型适配器超过 {timeout:g} 秒，本轮停止')
    if 'error' in result:
        raise result['error']
    return validate_task(result['task'])
