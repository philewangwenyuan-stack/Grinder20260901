"""External Catkin setup must not replace the supervisor's environment."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class LivoxEnvironmentTest(unittest.TestCase):
    def test_probe_and_launch_keep_main_workspace(self):
        launcher = (Path(__file__).resolve().parents[2] / 'start_grinder_super_lio_base.sh').read_text()
        probe = launcher.split('detect_livox_setup() {', 1)[1].split('\n}\n', 1)[0]
        command = launcher.split('  livox_command=()', 1)[1].split('  local_livox_launch=', 1)[0]
        with tempfile.TemporaryDirectory(prefix='livox environment ') as directory:
            root = Path(directory)
            setup = root / 'setup.bash'
            setup.write_text('export ROS_PACKAGE_PATH=livox\n')
            rospack = root / 'rospack'
            rospack.write_text('#!/bin/bash\n[[ "$ROS_PACKAGE_PATH" == livox ]] && echo /external/livox\n')
            rospack.chmod(0o755)
            env = dict(os.environ, PATH=str(root) + os.pathsep + os.environ['PATH'],
                       ROS_PACKAGE_PATH='grinder', LIVOX_SETUP=str(setup))
            script = 'set -euo pipefail\ndetect_livox_setup() {' + probe + '\n}\n'
            script += 'detect_livox_setup\n[[ "$ROS_PACKAGE_PATH" == grinder ]]\nlivox_command=()' + command
            script += '\n[[ "$local_livox_package" == /external/livox ]]\n'
            script += '[[ "$("${livox_command[@]}" bash -c \'echo "$ROS_PACKAGE_PATH"\')" == livox ]]\n'
            script += '[[ "$ROS_PACKAGE_PATH" == grinder ]]\n'
            subprocess.run(['bash', '-c', script], env=env, check=True, timeout=10)


if __name__ == '__main__':
    unittest.main()
