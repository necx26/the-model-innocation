#!/usr/bin/env python3
"""Offline, stdlib-only integrity check; optionally check the running HTTP endpoint."""
import argparse
import hashlib
import json
from pathlib import Path
from urllib.request import urlopen


def check(root, base_url=None):
    pack = root/'assets/voice'
    manifest = json.loads((pack/'manifest.json').read_text(encoding='utf-8'))
    assert manifest['language'] == 'zh-CN' and manifest['gender'] == 'female', '不是中文女声包'
    assert '\ufffd' not in json.dumps(manifest, ensure_ascii=False), '包含损坏的文本'
    total = 0
    for name, clip in manifest['clips'].items():
        assert clip['file'] == name+'.mp3', '非法文件名'
        data = (pack/clip['file']).read_bytes()
        assert len(data) == clip['bytes'] > 1000, '语音文件大小错误: '+name
        assert hashlib.sha256(data).hexdigest() == clip['sha256'], '语音文件校验失败: '+name
        if base_url:
            with urlopen(base_url.rstrip('/')+'/api/voice/'+clip['file'], timeout=10) as r:
                assert r.headers.get_content_type() == 'audio/mpeg'
                assert r.read() == data, 'HTTP 音频与本地不一致: '+name
        total += len(data)
    for name in ['player.js', '试听.html']:
        assert (pack/name).is_file(), '缺少语音组件: '+name
    assert (root/'语音试听.sh').is_file(), '缺少试听入口'
    if base_url:
        with urlopen(base_url.rstrip('/')+'/api/voice/manifest.json', timeout=10) as r:
            assert json.loads(r.read()) == manifest
        with urlopen(base_url.rstrip('/')+'/api/voice/player.js', timeout=10) as r:
            assert r.read() == (pack/'player.js').read_bytes()
    return dict(passed=True, voice=manifest['voice'], clips=len(manifest['clips']),
                bytes=total, offline_assets=True, http_checked=bool(base_url))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--url')
    args = parser.parse_args()
    print(json.dumps(check(args.root, args.url), ensure_ascii=False, indent=2))
