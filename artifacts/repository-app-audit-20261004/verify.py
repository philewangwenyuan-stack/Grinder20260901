"""Read-only source audit and offline tests. Never connects to a robot."""
import ast
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
APP = Path('E:/CODE/grindingrobot/grindingrobot')
OUT = Path(__file__).parent
def git(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args]).decode('utf-8', 'replace').strip()

tracked = git(ROOT, 'ls-files', '-z').split('\0')
prefixes = ('catkin_ws/src/grinder_', 'catkin_ws/scripts/',
            'catkin_ws/src/mapping/cloud_to_occupancy_grid/',
            'catkin_ws/src/localization/super_lio_loop/')
python_files = [p for p in tracked if p.endswith('.py') and p.startswith(prefixes)]
xml_files = [p for p in tracked if p.endswith(('.xml', '.launch')) and p.startswith(prefixes)]
errors = []
for p in python_files:
    try: ast.parse((ROOT/p).read_text(encoding='utf-8-sig'), filename=p, feature_version=(3, 8))
    except Exception as exc: errors.append([p, str(exc)])
for p in xml_files:
    try: ET.parse(ROOT/p)
    except Exception as exc: errors.append([p, str(exc)])

proto_a = ROOT/'third_party/sl_linka/proto/sl_link.proto'
proto_b = APP/'core/sllink/proto/sl_link.proto'
normalized = lambda p: p.read_text(encoding='utf-8-sig').replace('\r\n', '\n').encode('utf-8')
results = dict(robot_head=git(ROOT, 'rev-parse', 'HEAD'), app_head=git(APP, 'rev-parse', 'HEAD'),
               python38_files=len(python_files), xml_files=len(xml_files), syntax_errors=errors,
               protocol_equal=normalized(proto_a) == normalized(proto_b),
               proto_sha256=hashlib.sha256(normalized(proto_a)).hexdigest(), tests=[])
env = os.environ.copy()
env['PYTHONDONTWRITEBYTECODE'] = '1'
def run(name, args, path=None):
    current_env = dict(env)
    if path: current_env['PYTHONPATH'] = str(path)
    try:
        proc = subprocess.run(args, cwd=ROOT, env=current_env, text=True, encoding='utf-8', errors='replace',
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
        results['tests'].append(dict(name=name, returncode=proc.returncode, output=proc.stdout))
        print('{}: exit={}'.format(name, proc.returncode), flush=True)
    except Exception as exc:
        results['tests'].append(dict(name=name, error=str(exc)))
run('safety_cache', [sys.executable, str(ROOT/'artifacts/repository-audit-20260928/reproduce_safety_cache.py')])
run('scheduler_races', [sys.executable, str(OUT/'reproduce_scheduler_races.py')])
run('register_map', [sys.executable, '-m', 'unittest', 'discover', '-s',
                    'catkin_ws/src/grinder_chassis_driver/test', '-p', 'test_register_map.py'],
    ROOT/'catkin_ws/src/grinder_chassis_driver/src')
run('scheduler_config', [sys.executable, '-m', 'unittest', 'discover', '-s', 'catkin_ws/scripts/test',
                        '-p', 'test_scheduler_config.py'])
for test in sorted((ROOT/'catkin_ws/src/grinder_scheduler/test').glob('test_*.py')):
    run(test.name, [sys.executable, str(test)], ROOT/'catkin_ws/src/grinder_scheduler/src')
results['junit_direct_output'] = (ROOT/'.planning/repository-app-audit-20261004/direct-junit.log').read_text(encoding='utf-8', errors='replace')
(OUT/'verification.json').write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')
print('syntax Python={} XML={} errors={}; protocol_equal={}'.format(len(python_files), len(xml_files), len(errors), results['protocol_equal']))
