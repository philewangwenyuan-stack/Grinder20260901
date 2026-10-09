"""Clone a temporary candidate commit; validate without starting ROS/hardware."""
import ast
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import yaml

root = Path(sys.argv[1]).resolve()
snapshot = sys.argv[2]
checks = {}
output_dir = Path(__file__).resolve().parent
checkout_parent = Path(sys.argv[3]) if len(sys.argv) > 3 else Path(tempfile.mkdtemp(prefix="grinder-f04-f05-"))
clone = checkout_parent / "clone"


def run(args, **kwargs):
    result = subprocess.run(args, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, **kwargs)
    if result.returncode:
        raise RuntimeError("{} failed: {}".format(args[0], result.stdout[-2500:]))
    return result


print("Cloning candidate snapshot into {}".format(clone), flush=True)
if not clone.exists():
    run(["git", "clone", "--quiet", "--no-checkout", "--no-local", str(root), str(clone)])
run(["git", "-C", str(clone), "fetch", "--quiet", str(root), "refs/codex-validation/f04-f05"])
run(["git", "-C", str(clone), "checkout", "--quiet", "--detach", snapshot])
tracked = run(["git", "-C", str(clone), "ls-files", "--stage"]).stdout
assert not any(line.startswith("160000 ") for line in tracked.splitlines())
super_lio = clone / "catkin_ws/src/localization/Super-LIO"
assert not (super_lio / ".git").exists()
manifest = json.loads((root / ".git/local-backups/f04-f05-20260928/source-manifest.json").read_text())
assert all((super_lio / name).is_file() for name in manifest)
for name in manifest:
    if name == ".gitignore":
        continue
    original = (root / "catkin_ws/src/localization/Super-LIO" / name).read_bytes()
    recovered = (super_lio / name).read_bytes()
    assert original.replace(b"\r\n", b"\n") == recovered.replace(b"\r\n", b"\n"), name
assert (super_lio / "UPSTREAM.md").is_file()
assert (super_lio / "LICENSE").is_file()
assert (super_lio / "src/super_lio/srv/SetMap.srv").is_file()
assert not (super_lio / "src/super_lio/scripts/filtered_intensity_region.pcd").exists()
checks["source_recovery"] = {"original_files": len(manifest), "gitlinks": 0, "result": "passed"}

config_dir = clone / "catkin_ws/src/grinder_scheduler/config"
assert not (config_dir / "scheduler.local.yaml").exists()
public = yaml.safe_load((config_dir / "scheduler.yaml").read_text())
for key in ("file_upload_on_map_save", "mqtt_enabled", "stream_enabled"):
    assert public[key] is False
for key in ("platform_password", "mqtt_password", "platform_username", "mqtt_username", "mqtt_dev_code"):
    assert public[key] == ""
run([sys.executable, str(clone / "catkin_ws/scripts/test/test_scheduler_config.py")])
checks["configuration_tests"] = {"tests": 5, "result": "passed"}

shell_files = [clone / "catkin_ws" / name for name in (
    "start_grinder_stack.sh", "start_grinder_super_lio_base.sh", "start_grinder_sim.sh",
    "scripts/scheduler_config.sh")]
for file in shell_files:
    run(["bash", "-n", str(file)])
for file in [clone / "catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py",
             clone / "catkin_ws/scripts/check_scheduler_config.py",
             clone / "catkin_ws/scripts/test/test_scheduler_config.py"]:
    ast.parse(file.read_text(), filename=str(file), feature_version=(3, 8))
checks["syntax"] = {"bash": len(shell_files), "python38": 3, "result": "passed"}

# Check all CMake-declared message/service inputs in the newly recovered packages.
for package in (super_lio / "src/basic", super_lio / "src/super_lio"):
    cmake = (package / "CMakeLists.txt").read_text()
    for extension, folder in (("msg", "msg"), ("srv", "srv")):
        for name in re.findall(r"\b([A-Za-z0-9_]+\." + extension + r")\b", cmake):
            assert (package / folder / name).is_file(), name
    ET.parse(package / "package.xml")
checks["cmake_inputs"] = "passed"

os.environ["ROS_PACKAGE_PATH"] = str(clone / "catkin_ws/src") + ":/opt/ros/noetic/share"
os.environ.pop("GRINDER_SCHEDULER_LOCAL_CONFIG", None)
os.environ["ROS_HOME"] = str(checkout_parent / "ros-home")
import rospkg
import roslaunch

assert Path(rospkg.RosPack().get_path("super_lio")) == super_lio / "src/super_lio"
assert Path(rospkg.RosPack().get_path("basic")) == super_lio / "src/basic"


def load_launch(path, args=()):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(str(path), config, argv=list(args), verbose=False)
    return {key: param.value for key, param in config.params.items()}


scheduler_launch = clone / "catkin_ws/src/grinder_scheduler/launch/scheduler.launch"
sim_launch = clone / "catkin_ws/src/grinder_gazebo/launch/scheduler_sim.launch"
for launch in (scheduler_launch, sim_launch):
    params = load_launch(launch)
    for key in ("file_upload_on_map_save", "mqtt_enabled", "stream_enabled"):
        assert params["/grinder_scheduler/" + key] is False
override = checkout_parent / "test-local.yaml"
override.write_text("mqtt_enabled: true\nfile_upload_on_map_save: true\nstream_enabled: true\n"
                    "vehicle_width: 1.23\n", encoding="utf-8")
params = load_launch(scheduler_launch, ["scheduler_local_config:=" + str(override)])
assert params["/grinder_scheduler/mqtt_enabled"] is True
assert params["/grinder_scheduler/vehicle_width"] == 1.23
os.environ["GRINDER_SCHEDULER_LOCAL_CONFIG"] = str(override)
params = load_launch(scheduler_launch)
assert params["/grinder_scheduler/mqtt_enabled"] is True
os.environ.pop("GRINDER_SCHEDULER_LOCAL_CONFIG")
params = load_launch(sim_launch, ["scheduler_local_config:=" + str(override)])
for key in ("file_upload_on_map_save", "mqtt_enabled", "stream_enabled"):
    assert params["/grinder_scheduler/" + key] is False
for bad in (checkout_parent / "missing.yaml", checkout_parent / "broken.yaml"):
    if bad.name == "broken.yaml":
        bad.write_text("mqtt_password: [do-not-log\n", encoding="utf-8")
    try:
        load_launch(scheduler_launch, ["scheduler_local_config:=" + str(bad)])
    except Exception:
        pass
    else:
        raise AssertionError("invalid explicit local config was accepted")
checks["roslaunch_configuration"] = "passed: defaults, local arg/env, sim isolation, missing/invalid file"

outer_results = {}
for launch in (clone / "catkin_ws/src/grinder_scheduler/launch/grinder_system.launch",
               clone / "catkin_ws/src/grinder_gazebo/launch/grinder_sim.launch"):
    try:
        params = load_launch(launch, ["scheduler_local_config:=" + str(override)])
        expected = launch.name == "grinder_system.launch"
        assert params["/grinder_scheduler/mqtt_enabled"] is expected
        outer_results[launch.name] = "passed"
    except Exception as exc:
        # These larger graphs may require system-installed Gazebo/navigation deps.
        outer_results[launch.name] = type(exc).__name__ + ": " + str(exc).splitlines()[0]
checks["outer_launch"] = outer_results

# Verify wrapper policy with an artificial local file, without any runtime launch.
fake_ws = checkout_parent / "wrapper-fixture"
fake_config = fake_ws / "src/grinder_scheduler/config"
fake_config.mkdir(parents=True, exist_ok=True)
(fake_ws / "scripts").mkdir(exist_ok=True)
import shutil
shutil.copy2(clone / "catkin_ws/scripts/check_scheduler_config.py", fake_ws / "scripts")
(fake_config / "scheduler.yaml").write_text("mqtt_enabled: false\n")
(fake_config / "scheduler.local.yaml").write_text("mqtt_enabled: true\n")
helper = clone / "catkin_ws/scripts/scheduler_config.sh"
script = 'source "$1"; configure_scheduler_config "$2" "$3"; printf "LOCAL=%s\\n" "$GRINDER_SCHEDULER_LOCAL_CONFIG"'
env = os.environ.copy()
env.pop("GRINDER_SCHEDULER_LOCAL_CONFIG", None)
device = run(["bash", "-c", script, "check", str(helper), str(fake_ws), "device"], env=env).stdout
sim = run(["bash", "-c", script, "check", str(helper), str(fake_ws), "sim"], env=env).stdout
assert "LOCAL=" + str(fake_config / "scheduler.local.yaml") in device
assert sim.endswith("LOCAL=\n")
env["GRINDER_SCHEDULER_LOCAL_CONFIG"] = ""
assert run(["bash", "-c", script, "check", str(helper), str(fake_ws), "device"], env=env).stdout.endswith("LOCAL=\n")
checks["wrapper_policy"] = "passed: device discovery, simulation isolation, explicit empty opt-out"

print("Offline recovery and launch checks passed; attempting Catkin build.", flush=True)
build_log = checkout_parent / "catkin-build.log"
with build_log.open("w") as handle:
    proc = subprocess.run(["catkin_make", "-C", str(clone / "catkin_ws"),
                           "--only-pkg-with-deps", "grinder_scheduler", "-j2", "-l2"],
                          stdout=handle, stderr=subprocess.STDOUT)
checks["catkin_build"] = {"returncode": proc.returncode, "log": str(build_log)}
if proc.returncode:
    text = build_log.read_text(errors="replace")
    checks["catkin_build"]["tail"] = text[-6000:]
checks["candidate_commit"] = snapshot
checks["clean_clone"] = str(clone)
(output_dir / "verification.json").write_text(json.dumps(checks, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps(checks, ensure_ascii=False, indent=2), flush=True)
