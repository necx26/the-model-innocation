"""Shared scene data for reset, perception, routing, map generation and UI."""
import json
import os
from pathlib import Path

CONFIG_PATH = Path(os.environ.get('YZBOT_SCENE_CONFIG') or Path(__file__).with_name('scene_config.json'))
SCENE = json.loads(CONFIG_PATH.read_text())
ZONES = {key: tuple(map(float, value)) for key, value in SCENE['zones'].items()}
PATROLS = SCENE['patrols']


def in_delivery_area(point, extra=0.):
    return any(abs(point[0]-x) < .65+extra and abs(point[1]-y) < .40+extra for x, y in ZONES.values())
