#!/usr/bin/env python3
"""Regenerate the portable voice assets. Only this build step needs Internet/edge-tts."""
import argparse
import asyncio
import hashlib
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACK = ROOT / 'assets/voice'
VOICE = 'zh-CN-XiaoxiaoNeural'


def phrases():
    lines = {
        'enabled': '中文女声已开启。准备好啦，我们一起完成搬运任务吧！',
        'ready': '机器人准备好啦，可以开始考试了。',
        'solve': '收到题目啦！正在分析搬运要求，请稍等一下。',
        'search': '我来看看，下一个目标在哪里。',
        'approach': '找到目标啦！现在靠近，准备抓取。',
        'approach_blocked': '前方有障碍，我来换一条安全的路线。',
        'grasp_red': '准备抓取红色物体，稳稳拿好再出发。',
        'grasp_blue': '准备抓取蓝色物体，稳稳拿好再出发。',
        'grasp_target': '准备抓取目标物体，稳稳拿好再出发。',
        'carry_red': '红色物体抓稳啦！现在送往指定区域。',
        'carry_blue': '蓝色物体抓稳啦！现在送往指定区域。',
        'carry_target': '物体抓稳啦！现在送往指定区域。',
        'place': '到达放置区啦！轻轻放好，再后退离开。',
        'complete': '太棒啦！搬运任务完成，结果已经核验完毕！',
        'incomplete': '本轮任务还没有完成，请查看界面上的具体原因。',
        'stopped': '作业已停止。需要重新开始时，请先重置场景。',
        'prepare': '正在准备场景和导航，马上就好。',
        'reset_failed': '场景还没有准备好，请查看界面提示。',
        'waiting': '等待下一步任务。',
        'disconnected': '机器人连接中断啦，请检查仿真运行状态。',
    }
    for color, chinese in [('red', '红色'), ('blue', '蓝色'), ('target', '目标')]:
        for zone in 'ABC':
            lines[f'carry_{color}_{zone.lower()}'] = f'{chinese}物体抓稳啦！现在送往 {zone} 区。'
    return lines


async def build(force):
    import edge_tts
    PACK.mkdir(parents=True, exist_ok=True)
    previous = json.loads((PACK/'manifest.json').read_text()) if (PACK/'manifest.json').exists() else {}
    clips = {}
    gate = asyncio.Semaphore(3)

    async def render(key, text):
        bright = key in ('enabled', 'complete')
        rate, pitch = ('+5%', '+8Hz') if bright else ('+3%', '+5Hz')
        settings = dict(text=text, voice=VOICE, rate=rate, pitch=pitch)
        recipe = hashlib.sha256(json.dumps(settings, ensure_ascii=False, sort_keys=True).encode()).hexdigest()
        dest = PACK/(key+'.mp3')
        old = previous.get('clips', {}).get(key, {})
        valid = (not force and dest.exists() and old.get('recipe') == recipe
                 and hashlib.sha256(dest.read_bytes()).hexdigest() == old.get('sha256'))
        async with gate:
            if not valid:
                temp = dest.with_suffix('.part')
                for attempt in range(3):
                    try:
                        await asyncio.wait_for(edge_tts.Communicate(
                            **settings, proxy=os.getenv('HTTPS_PROXY') or os.getenv('https_proxy')
                        ).save(str(temp)), 40)
                        if temp.stat().st_size < 1000:
                            raise ValueError('empty audio: '+key)
                        temp.replace(dest)
                        break
                    except Exception:
                        temp.unlink(missing_ok=True)
                        if attempt == 2:
                            raise
                        await asyncio.sleep(attempt+1)
            clips[key] = dict(file=dest.name, text=text, recipe=recipe,
                              bytes=dest.stat().st_size, sha256=hashlib.sha256(dest.read_bytes()).hexdigest())
            print('OK', key, dest.stat().st_size, flush=True)

    await asyncio.gather(*(render(k, v) for k, v in phrases().items()))
    manifest = dict(schema=1, language='zh-CN', gender='female', voice=VOICE,
                    runtime='offline audio playback; no model or TTS service required',
                    delivery='温柔轻快；语句、标点、语速与音高调整；不使用情感风格 SSML 标签',
                    generator='edge-tts 7.3.1',
                    source='https://github.com/rany2/edge-tts',
                    clips={k: clips[k] for k in phrases()})
    (PACK/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print('Voice pack ready:', len(clips), 'clips', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--force', action='store_true')
    asyncio.run(build(parser.parse_args().force))
