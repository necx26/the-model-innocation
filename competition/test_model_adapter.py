"""Contract tests without ROS, a model, network, or robot movement."""
import os
import sys
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros_ws'))
from model_adapter import invoke_official, validate_task

TASK = dict(red=2, blue=3, first='red', red_zone='B', blue_zone='C')


class AdapterTests(unittest.TestCase):
    def test_unconfigured_official_fails(self):
        with patch.dict(os.environ, {'YZBOT_OFFICIAL_ADAPTER': 'official_model_adapter:solve_task',
                                     'YZBOT_OFFICIAL_CONFIG': ''}):
            with self.assertRaisesRegex(RuntimeError, '尚未实现'):
                invoke_official('官方题目', {})

    def test_custom_adapter_receives_question_context(self):
        module = types.ModuleType('test_official_provider')
        received = {}
        def solve_task(**kwargs):
            received.update(kwargs)
            return dict(TASK)
        module.solve_task = solve_task
        with patch.dict(sys.modules, {module.__name__: module}), patch.dict(os.environ, {
                'YZBOT_OFFICIAL_ADAPTER': module.__name__ + ':solve_task', 'YZBOT_OFFICIAL_CONFIG': ''}):
            self.assertEqual(invoke_official('真实输入', {'round_id': 'r1'}), TASK)
        self.assertEqual(received, {'question': '真实输入', 'context': {'round_id': 'r1'}, 'config': {}})

    def test_invalid_tasks_are_rejected(self):
        for changes in ({'red': True}, {'red': 1.0}, {'blue': -1}, {'red': 5},
                        {'first': 'green'}, {'red_zone': 'D'}, {'extra': 'ignored'}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_task({**TASK, **changes})
        with self.assertRaises(ValueError):
            validate_task({'red': 2})

    def test_question_context_required(self):
        with self.assertRaises(ValueError):
            invoke_official('', {})
        with self.assertRaises(ValueError):
            invoke_official('question', [])

    def test_inference_deadline_stops_mission(self):
        module = types.ModuleType('test_slow_provider')
        module.solve_task = lambda **kwargs: (time.sleep(.2), dict(TASK))[1]
        with patch.dict(sys.modules, {module.__name__: module}), patch.dict(os.environ, {
                'YZBOT_OFFICIAL_ADAPTER': module.__name__ + ':solve_task',
                'YZBOT_OFFICIAL_CONFIG': '', 'YZBOT_OFFICIAL_TIMEOUT': '.02'}):
            with self.assertRaises(TimeoutError):
                invoke_official('question', {})


if __name__ == '__main__':
    unittest.main()
