"""Exercise remote command construction without contacting SSH or a robot."""
import json
import os
import pathlib
import subprocess
import tempfile
import unittest

SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'remote_sim.sh'


class RemoteSimTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = pathlib.Path(self.tmp.name)
        self.log = self.root / 'calls.jsonl'
        key = self.root / 'identity'
        key.touch()
        for name in ('ssh', 'rsync'):
            executable = self.root / name
            executable.write_text(
                '#!/usr/bin/env python3\nimport os,sys,json\n'
                'with open(os.environ["TEST_CALL_LOG"],"a") as f:\n'
                ' f.write(json.dumps([os.path.basename(sys.argv[0])]+sys.argv[1:])+"\\n")\n')
            executable.chmod(0o755)
        self.env = dict(os.environ, PATH=str(self.root)+os.pathsep+os.environ['PATH'],
                        GRINDER_REMOTE_KEY=str(key), TEST_CALL_LOG=str(self.log), DISPLAY=':0')
        for name in ('GRINDER_REMOTE_HOST','GRINDER_REMOTE_USER','GRINDER_REMOTE_ROOT','GRINDER_REMOTE_PORT'):
            self.env.pop(name, None)

    def run_action(self, *args):
        result = subprocess.run(['bash', str(SCRIPT)]+list(args), env=self.env,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        calls = [json.loads(x) for x in self.log.read_text().splitlines()] if self.log.exists() else []
        return result, calls

    def test_sync_protects_remote_state_builds_keys_and_source_owner_marker(self):
        result, calls = self.run_action('sync')
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual(['ssh','rsync'], [c[0] for c in calls])
        self.assertIn('Refusing', calls[0][-1])
        args = calls[1]
        self.assertIn('--delete-delay', args)
        for excluded in ('/.grinder-source-mirror','temp/','build/','devel/','/maps/',
                         '.git/','*.so','*.pcd','scheduler.local.yaml'):
            self.assertIn('--exclude='+excluded, args)
        self.assertNotIn('--delete-excluded', args)
        self.assertIn('--include=/catkin_ws/src/2-dnavigation-package/aurora_remote_public/lib/***', args)
        self.assertTrue(args[-1].endswith('/Grinder-remote-sim/'))

    def test_unsafe_remote_root_is_rejected_before_ssh(self):
        for root in ('/','/home/dev','/home/dev/../../etc',"/home/dev/work/a';touch /tmp/a"):
            self.env['GRINDER_REMOTE_ROOT'] = root
            result, calls = self.run_action('sync')
            self.assertNotEqual(0, result.returncode)
            self.assertEqual([], calls)

    def test_build_also_rebuilds_native_planner(self):
        result, calls = self.run_action('build')
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual(['ssh','rsync','ssh'], [c[0] for c in calls])
        self.assertIn('PROFILE=sim', calls[-1][-1])
        self.assertIn('third_party/path_planner/mst27', calls[-1][-1])

    def test_start_defaults_to_remote_headless_gazebo(self):
        result, calls = self.run_action('start')
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn('gui:=false', calls[-1][-1])
        self.assertIn('local_rtsp_public_host:', calls[-1][-1])
        self.assertIn('127.0.0.1', calls[-1][-1])

    def test_rviz_uses_x11_and_remote_workspace(self):
        result, calls = self.run_action('rviz')
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn('-Y', calls[-1])
        self.assertIn('LIBGL_ALWAYS_SOFTWARE=1', calls[-1][-1])
        self.assertIn('rosrun', calls[-1][-1])

    def test_shell_sources_remote_ros_environment(self):
        result, calls = self.run_action('shell')
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn('-t', calls[-1])
        self.assertIn('/opt/ros/noetic/setup.bash', calls[-1][-1])
        self.assertIn('/catkin_ws/devel/setup.bash', calls[-1][-1])


if __name__ == '__main__':
    unittest.main()
