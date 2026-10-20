import os
import signal
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'native'))
from navigation_process import terminate_group


class NavigationOwnershipTests(unittest.TestCase):
    def test_unrelated_process_is_not_stopped(self):
        with tempfile.TemporaryDirectory() as project:
            child=subprocess.Popen(['sleep','30'],start_new_session=True)
            try:
                with self.assertRaises(RuntimeError):terminate_group(child.pid,project)
                self.assertIsNone(child.poll())
            finally:os.killpg(child.pid,signal.SIGTERM);child.wait()
    def test_own_project_group_is_stopped(self):
        with tempfile.TemporaryDirectory() as project:
            child=subprocess.Popen(['sleep','30'],start_new_session=True,
                                   env=dict(os.environ,YZBOT_PROJECT=project))
            try:
                terminate_group(child.pid,project)
                self.assertEqual(child.wait(timeout=2),-signal.SIGTERM)
            finally:
                if child.poll() is None:child.kill();child.wait()


if __name__=='__main__':unittest.main()
