"""Start 2-3 isolated Grinder stacks in one shared Gazebo physics world."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time

import yaml

from .fleet_config import isolate_urdf, pose, validate_fleet


def check_ports(config):
    ports = [config["world_master_port"], config["gazebo_master_port"]]
    for robot in config["robots"]:
        ports.extend(robot[key] for key in ("master_port", "app_port", "control_port"))
    for number in ports:
        with socket.socket() as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                listener.bind(("0.0.0.0", number))
            except OSError as exc:
                raise RuntimeError("TCP port {} already in use; existing simulations were not stopped".format(number)) from exc


def prepare_robot(workspace, package, robot, cameras, base_urdf):
    """Keep device writes out of shared source/config and out of other cars."""
    root = workspace / "temp" / "sim_fleet" / robot["name"]
    root.mkdir(parents=True, exist_ok=True)
    navigation = workspace / "src/2-dnavigation-package/2dnavigation/teb_local_planner_tutorials/cfg/diff_drive"
    for source, target in (("costmap_common_params.yaml", "costmap.yaml"),
                           ("rpp_local_planner_params.yaml", "rpp.yaml")):
        if not (root / target).exists():
            shutil.copyfile(str(navigation / source), str(root / target))
    (root / "robot.urdf").write_text(isolate_urdf(base_urdf, robot["name"], cameras), encoding="utf-8")
    paths = {
        "persist_state_dir": root / "state", "robot_config_yaml_path": root / "robot_config.yaml",
        "costmap_common_config_yaml_path": root / "costmap.yaml", "rpp_config_yaml_path": root / "rpp.yaml",
        "navigation_map_yaml_path": root / "navigation/map.yaml", "stcm_local_dir": root / "raw",
        "live_preview_file": root / "preview/latest.jpg", "planned_path_preview_dir": root / "preview",
        "planned_path_debug_dir": root / "path_debug", "first_frame_save_dir": root / "first_frame",
        "initial_map_preview_dir": root / "preview", "local_rtsp_log_dir": root / "video",
        "metrics_export_dir": root / "metrics",
    }
    overrides = {key: str(value) for key, value in paths.items()}
    overrides.update(sl_linka_port=robot["app_port"], sl_linka_control_port=robot["control_port"],
                     image_inputs_enabled=cameras, local_rtsp_enabled=cameras,
                     local_rtsp_start_server=cameras, local_rtsp_port=robot["rtsp_port"],
                     mqtt_enabled=False, file_upload_on_map_save=False, stream_enabled=False)
    (root / "runtime.yaml").write_text(yaml.safe_dump(overrides), encoding="utf-8")
    maps = workspace.parent / "maps" / "sim_fleet" / robot["name"]
    maps.mkdir(parents=True, exist_ok=True)
    return root, maps


class Supervisor:
    def __init__(self):
        self.jobs = []
        self.stopping = False

    def start(self, name, argv, env, directory):
        directory.mkdir(parents=True, exist_ok=True)
        logfile = open(str(directory / (name + ".log")), "ab", buffering=0)
        try:
            process = subprocess.Popen(argv, env=env, stdout=logfile, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        finally:
            logfile.close()
        self.jobs.append((name, process))
        return process

    def check(self):
        if self.stopping:
            raise KeyboardInterrupt()
        for name, process in self.jobs:
            if process.poll() is not None:
                raise RuntimeError("{} exited ({}); inspect its fleet log".format(name, process.returncode))

    def wait_master(self, number):
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            self.check()
            try:
                with socket.create_connection(("127.0.0.1", number), timeout=0.2):
                    return
            except OSError:
                time.sleep(0.1)
        raise RuntimeError("ROS master did not start: " + str(number))

    def stop(self):
        def interrupt(jobs):
            for _, process in reversed(jobs):
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
        # Leave masters and bridges alive while managers stop owned LIO nodes.
        stacks = [job for job in self.jobs if job[0].endswith("_stack")]
        interrupt(stacks)
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline and any(p.poll() is None for _, p in stacks):
            time.sleep(0.1)
        interrupt(self.jobs)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and any(p.poll() is None for _, p in self.jobs):
            time.sleep(0.1)
        for _, process in reversed(self.jobs):
            # Also reap grandchildren (e.g. a managed LIO launch) that survived
            # their group leader; never kill nodes/processes outside this fleet.
            for sig in (signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                time.sleep(0.1)
            process.wait()


def ros_environment(port, root, gazebo_port):
    env = os.environ.copy()
    env.update(ROS_MASTER_URI="http://127.0.0.1:{}/".format(port),
               GAZEBO_MASTER_URI="http://127.0.0.1:{}".format(gazebo_port),
               ROS_HOSTNAME="127.0.0.1", ROS_HOME=str(root / "ros_home"),
               ROS_LOG_DIR=str(root / "ros_logs"))
    env.pop("ROS_IP", None)
    env.pop("ROS_NAMESPACE", None)
    (root / "ros_logs").mkdir(parents=True, exist_ok=True)
    return env


def stop_existing(workspace):
    pidfile = workspace / "temp/sim_fleet/fleet.pid"
    if not pidfile.exists():
        print("[FLEET] no running fleet PID file")
        return
    pid = int(pidfile.read_text())
    try:
        argv = Path("/proc/{}/cmdline".format(pid)).read_bytes().decode().split("\0")
    except FileNotFoundError:
        raise RuntimeError("stale fleet.pid; no process was signalled")
    # Never signal an unrelated process after a reboot or PID reuse.
    if (not any(Path(arg).name == "start_fleet.py" for arg in argv if arg)
            or "--workspace" not in argv
            or Path(argv[argv.index("--workspace") + 1]).resolve() != workspace):
        raise RuntimeError("PID does not belong to this workspace fleet; no process was signalled")
    os.kill(pid, signal.SIGINT)
    deadline = time.monotonic() + 25
    while pidfile.exists() and time.monotonic() < deadline:
        time.sleep(0.1)
    if pidfile.exists():
        raise RuntimeError("fleet is still stopping; inspect fleet logs")
    print("[FLEET] stopped")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--config")
    parser.add_argument("--robots", type=int, choices=(2, 3), default=2)
    parser.add_argument("--gui", choices=("true", "false"))
    parser.add_argument("--sim-speed", type=float)
    parser.add_argument("--world")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--stop", action="store_true", help="stop only this workspace's fleet")
    args = parser.parse_args()
    workspace = Path(args.workspace).resolve()
    if args.stop:
        stop_existing(workspace)
        return 0
    package = workspace / "src/grinder_gazebo"
    config_path = Path(args.config).resolve() if args.config else package / "config/fleet.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if args.gui is not None:
        config["gui"] = args.gui == "true"
    if args.sim_speed is not None:
        config["sim_speed"] = args.sim_speed
    base = [0, 0, 0]
    spawn_file = workspace / "temp/sim_spawn_pose"
    if spawn_file.exists():
        base = pose(spawn_file.read_text().split())
    config = validate_fleet(config, args.robots, base)
    world = args.world or config.get("world") or os.environ.get("GRINDER_SIM_WORLD")
    world_file = workspace / "temp/sim_world_path"
    if not world and world_file.exists():
        world = world_file.read_text().strip()
    world = Path(world).expanduser() if world else package / "worlds/factory_floor.world"
    if not world.is_absolute():
        world = config_path.parent / world
    world = world.resolve()
    if not world.is_file():
        raise ValueError("world file is missing: " + str(world))
    config["world"] = str(world)
    # Optional camera streams also need unique TCP ports.
    ports = {config["world_master_port"], config["gazebo_master_port"]}
    ports.update(r[key] for r in config["robots"] for key in ("master_port", "app_port", "control_port"))
    for index, robot in enumerate(config["robots"]):
        from .fleet_config import port
        robot["rtsp_port"] = port(robot.get("rtsp_port", 8654 + index * 100))
        if config.get("cameras", False):
            if robot["rtsp_port"] in ports:
                raise ValueError("duplicate RTSP port")
            ports.add(robot["rtsp_port"])
        local = robot.get("local_config", "")
        if local:
            local = Path(local).expanduser()
            if not local.is_absolute():
                local = config_path.parent / local
            if not local.is_file():
                raise ValueError("local config is missing: " + str(local))
            robot["local_config"] = str(local.resolve())
    if args.dry_run:
        print(json.dumps(config, indent=2))
        return 0
    import fcntl
    runtime = workspace / "temp/sim_fleet"
    runtime.mkdir(parents=True, exist_ok=True)
    lockfile = open(str(runtime / "fleet.lock"), "a")
    fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
    check_ports(config)
    if config.get("cameras", False):
        for robot in config["robots"]:
            with socket.socket() as sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind(("0.0.0.0", robot["rtsp_port"]))
    models = world.parent.parent / "models"
    if models.is_dir():
        os.environ["GAZEBO_MODEL_PATH"] = str(models) + os.pathsep + os.environ.get("GAZEBO_MODEL_PATH", "")
    xacro = ["rosrun", "xacro", "xacro", str(package / "urdf/x920_grinder.urdf.xacro")]
    for key in ("lidar_backend", "lidar_horizontal_samples", "lidar_vertical_samples", "lidar_max_range"):
        xacro.append("{}:={}".format(key, config.get(key, "gpu")))
    base_urdf = subprocess.check_output(xacro).decode("utf-8")
    supervisor = Supervisor()
    def stop(_sig, _frame):
        supervisor.stopping = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    root_env = ros_environment(config["world_master_port"], runtime / "world", config["gazebo_master_port"])
    manifest = dict(config, workspace=str(workspace), pid=os.getpid())
    (runtime / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    try:
        (runtime / "fleet.pid").write_text(str(os.getpid()), encoding="ascii")
        supervisor.start("master", ["roscore", "-p", str(config["world_master_port"])], root_env, runtime / "world")
        supervisor.wait_master(config["world_master_port"])
        supervisor.start("world", ["roslaunch", "grinder_gazebo", "fleet_world.launch",
                                  "world_name:=" + str(world), "gui:=" + str(config.get("gui", True)).lower(),
                                  "sim_speed:=" + str(config["sim_speed"])], root_env, runtime / "world")
        for robot in config["robots"]:
            supervisor.check()
            name = robot["name"]
            root, maps = prepare_robot(workspace, package, robot, config.get("cameras", False), base_urdf)
            env = ros_environment(robot["master_port"], root, config["gazebo_master_port"])
            supervisor.start(name + "_master", ["roscore", "-p", str(robot["master_port"])], env, root)
            supervisor.wait_master(robot["master_port"])
            # spawn_model waits for Gazebo's factory service. Its URDF describes
            # the same physical world; no per-car Gazebo server is started.
            spawn_log = open(str(root / "spawn.log"), "ab")
            try:
                spawn = subprocess.Popen(["rosrun", "gazebo_ros", "spawn_model", "-urdf", "-file", str(root / "robot.urdf"),
                                          "-model", name, "-x", str(robot["spawn"][0]), "-y", str(robot["spawn"][1]),
                                          "-z", "0.1675", "-Y", str(robot["spawn"][2])], env=root_env,
                                         stdout=spawn_log, stderr=subprocess.STDOUT, start_new_session=True)
                supervisor.jobs.append((name + "_spawn", spawn))
                deadline = time.monotonic() + 90
                while spawn.poll() is None:
                    # A completed one-shot spawn is not a long-lived service.
                    if supervisor.stopping or time.monotonic() > deadline:
                        raise RuntimeError("model spawn interrupted/timed out: " + name)
                    for job, process in supervisor.jobs[:-1]:
                        if process.poll() is not None:
                            raise RuntimeError(job + " exited during spawn")
                    time.sleep(0.1)
                if spawn.returncode:
                    raise RuntimeError("model spawn failed: " + name + "; inspect spawn.log")
                supervisor.jobs.pop()
            finally:
                spawn_log.close()
            bridge = ["rosrun", "grinder_gazebo", "fleet_bridge.py", "--name", name,
                      "--world-uri", root_env["ROS_MASTER_URI"], "--robot-uri", env["ROS_MASTER_URI"]]
            if config.get("cameras", False):
                bridge.append("--cameras")
            supervisor.start(name + "_bridge", bridge, env, root)
            supervisor.start(name + "_stack", ["roslaunch", "grinder_gazebo", "fleet_robot.launch",
                             "robot_description_file:=" + str(root / "robot.urdf"), "runtime_config:=" + str(root / "runtime.yaml"),
                             "runtime_dir:=" + str(root), "map_root:=" + str(maps), "app_port:=" + str(robot["app_port"]),
                             "scheduler_local_config:=" + robot.get("local_config", ""),
                             "cameras:=" + str(config.get("cameras", False)).lower()], env, root)
            print("[FLEET] {} APP {} / control {} / ROS {} / spawn {}".format(
                name, robot["app_port"], robot["control_port"], robot["master_port"], robot["spawn"]), flush=True)
        deadline = time.monotonic() + 45
        pending = {r[key] for r in config["robots"] for key in ("app_port", "control_port")}
        while pending:
            supervisor.check()
            for number in list(pending):
                try:
                    with socket.create_connection(("127.0.0.1", number), timeout=0.1):
                        pending.remove(number)
                except OSError:
                    pass
            if time.monotonic() >= deadline:
                raise RuntimeError("APP listeners did not start: " + str(sorted(pending)))
            time.sleep(0.1)
        print("[FLEET] shared world ready; Ctrl+C stops this fleet only. Logs: " + str(runtime), flush=True)
        while not supervisor.stopping:
            supervisor.check()
            time.sleep(0.2)
    finally:
        try:
            supervisor.stop()
        finally:
            try:
                (runtime / "fleet.pid").unlink()
            except FileNotFoundError:
                pass
            lockfile.close()
    return 0
