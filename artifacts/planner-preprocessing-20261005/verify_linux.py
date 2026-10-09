"""Run current offline tests with the freshly compiled native C++ extension."""
import importlib.util
import json
import pathlib
import sys
import unittest

here = pathlib.Path(__file__).resolve().parent
root = here.parents[1]
sys.path.insert(0,str(here/'native'))
import _mst27_cpp

suite = unittest.TestSuite()
paths = [root/'third_party/path_planner/mst27/test/test_preprocessing.py',
         root/'catkin_ws/src/grinder_scheduler/test/test_planner_preprocessing.py',
         root/'catkin_ws/src/grinder_scheduler/test/test_planning_direction.py']
for index,path in enumerate(paths):
    spec = importlib.util.spec_from_file_location('offline_suite_{}'.format(index),str(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(module))
result = unittest.TextTestRunner(verbosity=2).run(suite)
report = dict(python=sys.version,native_module=_mst27_cpp.__file__,tests=result.testsRun,
    failures=[str(t) for t,_ in result.failures],errors=[str(t) for t,_ in result.errors],
    skipped=[str(t) for t,_ in result.skipped],passed=result.wasSuccessful())
(here/'linux_verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
raise SystemExit(not result.wasSuccessful())
