"""Offline service/launcher tests; no ROS nodes or hardware are opened."""
import importlib.machinery
import os
from pathlib import Path
import pwd
import signal
import shlex
import subprocess
import tempfile
import time
import unittest
from unittest import mock

HERE = Path(__file__).resolve().parent
manager = importlib.machinery.SourceFileLoader('grinder_service', str(HERE / 'grinder-service')).load_module()


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='grinder service ')
        self.root = Path(self.temporary.name)
        self.patches = [mock.patch.object(manager, key, self.root / name) for key, name in
                        [('CONFIG', 'etc/startup.conf'), ('UNIT', 'grinder.service'),
                         ('PROGRAM', 'lib/grinder-service'), ('COMMAND', 'bin/grinder-service')]]
        for patch in self.patches:
            patch.start()
        self.addCleanup(lambda: [patch.stop() for patch in reversed(self.patches)])
        self.addCleanup(self.temporary.cleanup)
        self.account = pwd.getpwnam('nobody')
        self.values = dict.fromkeys(manager.KEYS, '')
        self.values.update(AUTOSTART='0', RUN_USER=self.account.pw_name,
                           WORKSPACE=str(self.root / 'workspace with spaces'),
                           NAV_MAP_YAML=str(self.root / 'map with spaces.yaml'))

    def test_config_roundtrip_is_data(self):
        self.values['INDICATOR_USB_SERIAL_NUMBER'] = '$(touch /tmp/not-executed); `id`'
        manager.save_config(self.values)
        self.assertEqual(manager.read_config()[0], self.values)

    def test_invalid_config(self):
        manager.save_config(self.values)
        with manager.CONFIG.open('a') as stream:
            stream.write('AUTOSTART=1\n')
        with self.assertRaises(ValueError):
            manager.read_config()

    def test_apply_enable_disable_never_starts(self):
        with mock.patch.object(manager, 'systemctl') as control, \
                mock.patch.object(manager, 'check_runtime'):
            for value, operation in [('1', 'enable'), ('0', 'disable')]:
                self.values['AUTOSTART'] = value
                manager.apply(self.values, self.account)
                self.assertEqual(control.call_args, mock.call(operation, 'grinder.service'))
            self.assertFalse(any(call.args[0] in ('start', 'stop', 'restart') for call in control.call_args_list))
        unit = manager.UNIT.read_text()
        self.assertIn('Restart=no', unit)
        self.assertIn('KillMode=control-group', unit)
        self.assertIn('HOME=' + self.account.pw_dir, unit)

    def test_reinstall_preserves_config(self):
        manager.save_config(self.values)
        before = manager.CONFIG.read_bytes()
        with mock.patch.object(manager, 'systemctl'):
            manager.install('/different/workspace', 'different-user')
        self.assertEqual(manager.CONFIG.read_bytes(), before)

    def test_unit_definition(self):
        import configparser
        definition = manager.unit_text(self.values, self.account)
        parser = configparser.ConfigParser(interpolation=None)
        parser.read_string(definition)
        self.assertEqual(parser['Service']['User'], self.account.pw_name)
        self.assertEqual(parser['Service']['SupplementaryGroups'], 'dialout')
        self.assertEqual(parser['Install']['WantedBy'], 'multi-user.target')
        manager.atomic_write(manager.PROGRAM, '#!/bin/sh\nexit 0\n', 0o755)
        manager.atomic_write(manager.UNIT, definition)
        result = subprocess.run(['systemd-analyze', 'verify', str(manager.UNIT)],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if result.returncode and 'Failed to determine supported controllers' in result.stderr:
            self.skipTest('systemd verification unavailable: WSL lacks cgroup controllers; INI checks passed')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_usb_identity_validation(self):
        with mock.patch.object(Path, 'is_file', return_value=True):
            with self.assertRaises(ValueError):
                manager.check_runtime(self.values)
            self.values['INDICATOR_USB_SERIAL_NUMBER'] = 'controller123'
            manager.check_runtime(self.values)
            self.values['INDICATOR_DEVICE'] = '/dev/ttyACM0'
            with self.assertRaises(ValueError):
                manager.check_runtime(self.values)

    def test_commands_persist_boot_switch_without_runtime_actions(self):
        manager.save_config(self.values)
        with mock.patch.object(manager, 'management_lock', side_effect=context_lock), \
                mock.patch.object(manager.os, 'geteuid', return_value=0), \
                mock.patch.object(manager, 'check_runtime'), \
                mock.patch.object(manager, 'systemctl') as control:
            for operation, expected in [('enable', '1'), ('disable', '0')]:
                with mock.patch('sys.argv', ['grinder-service', operation]):
                    manager.main()
                self.assertEqual(manager.read_config()[0]['AUTOSTART'], expected)
            self.assertFalse(any(call.args[0] in ('start', 'stop', 'restart') for call in control.call_args_list))

    def test_stop_does_not_require_valid_config(self):
        with mock.patch.object(manager, 'management_lock', side_effect=context_lock), \
                mock.patch.object(manager.os, 'geteuid', return_value=0), \
                mock.patch.object(manager, 'systemctl') as control, \
                mock.patch('sys.argv', ['grinder-service', 'stop']):
            manager.main()
            control.assert_called_once_with('stop', 'grinder.service')

    def test_run_uses_account_home_and_full_stack_environment(self):
        manager.save_config(self.values)
        with mock.patch.object(manager.os, 'getuid', return_value=self.account.pw_uid), \
                mock.patch.object(manager, 'check_runtime'), \
                mock.patch.object(manager.os, 'chdir'), \
                mock.patch.object(manager.os, 'execvpe', side_effect=SystemExit) as execute, \
                mock.patch('sys.argv', ['grinder-service', 'run']):
            with self.assertRaises(SystemExit):
                manager.main()
            environment = execute.call_args.args[2]
            self.assertEqual(environment['HOME'], self.account.pw_dir)
            self.assertEqual(environment['INDICATOR_BACKEND'], 'serial')
            self.assertEqual(environment['START_INDICATOR'], '1')
            self.assertEqual(environment['START_LIVOX'], '1')


def context_lock():
    import contextlib
    return contextlib.nullcontext()


class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='grinder launch ')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / 'scripts').mkdir()
        (self.root / 'devel').mkdir()
        (self.root / 'devel/setup.bash').write_text('true\n')
        (self.root / 'setup.bash').write_text('true\n')
        (self.root / 'scripts/scheduler_config.sh').write_text('configure_scheduler_config() { true; }\n')
        text = (HERE.parent / 'start_grinder_super_lio_base.sh').read_text()
        text = text.replace('/opt/ros/noetic/setup.bash', shlex.quote(str(self.root / 'setup.bash')))
        self.script = self.root / 'start.sh'
        self.script.write_text(text)
        self.binary = self.root / 'bin'
        self.binary.mkdir()
        commands = {
            'pgrep': '#!/bin/sh\nexit 0\n',
            'rosparam': '#!/bin/sh\nexit 0\n',
            'roslaunch': '''#!/usr/bin/env python3
import os, sys, time
from pathlib import Path
with open(os.environ['MOCK_RECORD'], 'a') as stream:
    stream.write(repr((os.getpid(), sys.argv[1:])) + '\\n')
if os.environ.get('MOCK_FAIL') and 'chassis_driver.launch' in sys.argv:
    sys.exit(1)
while True:
    time.sleep(1)
'''}
        for name, content in commands.items():
            path = self.binary / name
            path.write_text(content)
            path.chmod(0o755)
        self.environment = os.environ.copy()
        self.environment.update(PATH=str(self.binary) + ':' + os.environ['PATH'],
                                WORKSPACE=str(self.root), START_LIVOX='0', START_INDICATOR='1',
                                INDICATOR_BACKEND='mock', MOCK_RECORD=str(self.root / 'record'),
                                NAV_MAP_YAML=str(self.root / 'map with spaces.yaml'))
        self.processes = []
        self.addCleanup(self.stop_processes)

    def launch(self):
        process = subprocess.Popen(['bash', str(self.script)], env=self.environment,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.processes.append(process)
        return process

    def stop_processes(self):
        for process in self.processes:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=15)

    def recorded(self):
        import ast
        record = self.root / 'record'
        return [ast.literal_eval(line) for line in record.read_text().splitlines()] if record.exists() else []

    def wait_record(self, count):
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if len(self.recorded()) >= count:
                return
            time.sleep(.1)
        self.fail('Mock launch did not reach expected process count')

    def assert_children_gone(self):
        for pid, _ in self.recorded():
            with self.assertRaises(ProcessLookupError):
                os.kill(pid, 0)

    def test_stop_duplicate_lock_and_argument_spaces(self):
        process = self.launch()
        self.wait_record(4)
        duplicate = self.launch()
        self.assertNotEqual(duplicate.wait(timeout=5), 0)
        entries = self.recorded()
        self.assertIn('status_indicator.launch', entries[0][1])
        self.assertIn('navigation_map_yaml_path:=' + self.environment['NAV_MAP_YAML'], entries[-1][1])
        process.send_signal(signal.SIGTERM)
        self.assertEqual(process.wait(timeout=15), 143)
        self.assert_children_gone()

    def test_early_failure_cleans_prior_children(self):
        self.environment['MOCK_FAIL'] = '1'
        process = self.launch()
        self.assertNotEqual(process.wait(timeout=20), 0)
        self.assert_children_gone()

    def test_runtime_child_exit_cleans_stack(self):
        process = self.launch()
        self.wait_record(4)
        # Let the launch startup delay finish before interrupting a child.
        time.sleep(2.2)
        os.kill(self.recorded()[1][0], signal.SIGTERM)
        self.assertNotEqual(process.wait(timeout=15), 0)
        self.assert_children_gone()


if __name__ == '__main__':
    unittest.main()
