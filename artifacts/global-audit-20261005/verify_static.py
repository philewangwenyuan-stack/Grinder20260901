"""Syntax/format and protocol consistency; does not import ROS or emit pyc."""
import ast
import hashlib
import json
import pathlib
import xml.etree.ElementTree as ET

root = pathlib.Path(__file__).resolve().parents[2]
package_roots = [root/'catkin_ws/src/grinder_scheduler',root/'catkin_ws/src/grinder_chassis_driver',root/'catkin_ws/src/grinder_gazebo']
python_paths = sorted({p for directory in package_roots[:2] for p in directory.rglob('*.py') if not any(part in ('build','devel','__pycache__') for part in p.parts)})
python_paths += [root/'third_party/path_planner/mst27/mst27.py',root/'third_party/path_planner/hybrid_planner_adaptive.py']
xml_paths = sorted({p for directory in package_roots for p in directory.rglob('*.launch')})
errors = []
for path in python_paths:
    try:
        ast.parse(path.read_text(encoding='utf-8-sig'), filename=str(path), feature_version=(3,8))
    except Exception as exc:
        errors.append(dict(file=str(path),error=str(exc)))
for path in xml_paths:
    try:
        ET.parse(path)
    except Exception as exc:
        errors.append(dict(file=str(path),error=str(exc)))
robot_proto = root/'third_party/sl_linka/proto/sl_link.proto'
app_proto = pathlib.Path('E:/CODE/grindingrobot/grindingrobot/core/sllink/proto/sl_link.proto')
def normalized(path):
    return path.read_text(encoding='utf-8-sig').replace('\r\n','\n').encode('utf-8')
result = dict(python38_files=len(python_paths),launch_xml_files=len(xml_paths),errors=errors,proto_equal=normalized(robot_proto)==normalized(app_proto),proto_sha256=hashlib.sha256(normalized(robot_proto)).hexdigest())
target = pathlib.Path(__file__).with_name('static_verification.json')
target.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
raise SystemExit(bool(errors) or not result['proto_equal'])
