"""Offline audit checks. Does not start ROS or access hardware/network."""
import ast
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[2]
os.chdir(root)
tracked = subprocess.check_output(["git", "ls-files", "-z"]).decode().split("\0")
prefixes = ("catkin_ws/src/grinder_", "catkin_ws/src/localization/super_lio_loop/",
            "catkin_ws/src/mapping/cloud_to_occupancy_grid/", "catkin_ws/scripts/")
py_files = [p for p in tracked if p.endswith(".py") and p.startswith(prefixes)]
syntax_errors = []
for p in py_files:
    try:
        ast.parse(Path(p).read_text(encoding="utf-8-sig"), filename=p, feature_version=(3, 8))
    except Exception as exc:
        syntax_errors.append([p, str(exc)])
xml_files = [p for p in tracked if p.startswith(prefixes) and
             (p.endswith(".launch") or p.endswith("package.xml"))]
xml_errors = []
for p in xml_files:
    try:
        ET.parse(p)
    except Exception as exc:
        xml_errors.append([p, str(exc)])
results = {"python38_syntax": {"files": len(py_files), "errors": syntax_errors},
           "xml": {"files": len(xml_files), "errors": xml_errors}, "tests": []}
env = os.environ.copy()
env["PYTHONDONTWRITEBYTECODE"] = "1"
env["PYTHONPATH"] = str(root / "catkin_ws/src/grinder_scheduler/src")
for test in sorted((root / "catkin_ws/src/grinder_scheduler/test").glob("test_*.py")):
    proc = subprocess.run([sys.executable, str(test)], env=env, text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
    results["tests"].append({"file": str(test.relative_to(root)), "returncode": proc.returncode,
                             "output": proc.stdout})
if sys.platform == "linux":
    scripts = [p for p in tracked if p.startswith("catkin_ws/") and p.endswith(".sh")
               and "/src/" not in p]
    results["bash_syntax"] = {"files": len(scripts), "errors": []}
    for script in scripts:
        proc = subprocess.run(["bash", "-n", script], text=True, capture_output=True)
        if proc.returncode:
            results["bash_syntax"]["errors"].append([script, proc.stderr])
out = Path(__file__).with_name("verification.json")
out.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps(results, ensure_ascii=False, indent=2))
