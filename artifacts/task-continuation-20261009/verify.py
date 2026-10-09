"""Offline Linux verification with the previously built native planner kernel."""
import json
import pathlib
import sys
import unittest

here = pathlib.Path(__file__).resolve().parent
root = here.parents[1]
sys.path.insert(0, str(root / 'artifacts/planner-preprocessing-20261005/native'))
sys.path.insert(0, str(root / 'catkin_ws/src/grinder_scheduler/test'))
sys.path.insert(0, str(root / 'third_party/path_planner/mst27/test'))
import _mst27_cpp

suite = unittest.defaultTestLoader.loadTestsFromNames([
    'test_task_continuation', 'test_planner_continuation',
    'test_planner_preprocessing', 'test_planning_direction', 'test_preprocessing'])
result = unittest.TextTestRunner(verbosity=2).run(suite)
report = dict(python=sys.version, native_module=_mst27_cpp.__file__, tests=result.testsRun,
              passed=result.wasSuccessful(), skipped=[str(t) for t, _ in result.skipped],
              failures=[str(t) for t, _ in result.failures], errors=[str(t) for t, _ in result.errors])
(here / 'verification.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
raise SystemExit(not result.wasSuccessful())
