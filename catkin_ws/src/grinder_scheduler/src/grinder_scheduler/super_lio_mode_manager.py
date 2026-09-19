#!/usr/bin/env python3

import json
import os
import queue
import re
import shutil
import threading
import time

import roslaunch
import rosnode
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse

from grinder_scheduler.srv import (
    GetSuperLioStatus,
    GetSuperLioStatusResponse,
    SaveSuperLioMap,
    SaveSuperLioMapResponse,
    StartSuperLioLocalization,
    StartSuperLioLocalizationResponse,
)
from super_lio.srv import GetMap, GetMapRequest


class SuperLioModeManager:
    IDLE = "IDLE"
    MAPPING = "MAPPING"
    SAVING = "SAVING"
    LOCALIZING = "LOCALIZING"
    ERROR = "ERROR"

    _MAPPING_NODES = {"/super_lio_node", "/cloud_to_occupancy_grid", "/super_lio_loop"}
    _LOCALIZATION_NODES = {"/relocation_node", "/map_server"}

    def __init__(self):
        self._main_thread_id = threading.get_ident()
        self._operation_queue = queue.Queue()
        self._lock = threading.RLock()
        self._state = self.IDLE
        self._message = "idle"
        self._active_map_id = ""
        self._bundle_dir = ""
        self._localization_ready = False
        self._initial_pose_received = False
        self._odom_event = threading.Event()
        self._map_event = threading.Event()
        self._launch_parent = None
        self._owned_mode = ""
        self._map_root = os.path.abspath(os.path.expanduser(rospy.get_param(
            "~super_lio_map_root", "/home/neardi/work/Grinder/maps"
        )))
        if self._map_root in (os.path.abspath(os.sep), os.path.abspath(os.path.expanduser("~"))):
            raise RuntimeError("super_lio_map_root must be a dedicated map directory")
        self._startup_timeout = max(
            2.0, float(rospy.get_param("~super_lio_startup_timeout_sec", 60.0))
        )
        self._shutdown_timeout = max(
            1.0, float(rospy.get_param("~super_lio_shutdown_timeout_sec", 8.0))
        )
        self._loop_closure = bool(rospy.get_param("~super_lio_loop_closure", False))
        self._autosave = bool(rospy.get_param("~super_lio_autosave", False))
        self._mapping_launch = roslaunch.rlutil.resolve_launch_arguments(
            ["grinder_scheduler", "super_lio_mapping_managed.launch"]
        )[0]
        self._localization_launch = roslaunch.rlutil.resolve_launch_arguments(
            ["grinder_scheduler", "super_lio_localization_managed.launch"]
        )[0]
        self._task_enable_pub = rospy.Publisher(
            "/chassis/task_enable", Bool, queue_size=1, latch=True
        )
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self._initial_pose_callback, queue_size=1)
        rospy.Subscriber("/lio/odom", Odometry, self._localization_odom_callback, queue_size=1)
        rospy.Subscriber("/map", OccupancyGrid, self._map_callback, queue_size=1)
        os.makedirs(self._map_root, exist_ok=True)
        os.makedirs(self._staging_dir(), exist_ok=True)
        self._detect_unowned_conflicts()

        rospy.Service("/super_lio_mode/start_mapping", Trigger, self._service_start_mapping)
        rospy.Service("/super_lio_mode/save_map", SaveSuperLioMap, self._service_save_map)
        rospy.Service(
            "/super_lio_mode/start_localization",
            StartSuperLioLocalization,
            self._service_start_localization,
        )
        rospy.Service("/super_lio_mode/stop", Trigger, self._service_stop)
        rospy.Service("/super_lio_mode/get_status", GetSuperLioStatus, self._handle_status)
        rospy.on_shutdown(self.shutdown)
        self._publish_safe()
        rospy.loginfo(
            "Super-LIO mode manager ready: state=%s map_root=%s", self._state, self._map_root
        )

    def _call_on_main_thread(self, handler, request):
        """Run roslaunch operations on the Python main thread.

        rospy services execute in worker threads. roslaunch installs Unix signal
        handlers while creating its process monitor, and Python only permits
        signal registration from the main thread. Queue the complete service
        operation and keep the service callback blocked until the main loop has
        produced its response.
        """
        if threading.get_ident() == self._main_thread_id:
            return handler(request)
        operation = {
            "handler": handler,
            "request": request,
            "event": threading.Event(),
            "response": None,
            "error": None,
        }
        self._operation_queue.put(operation)
        while not operation["event"].wait(0.1):
            if rospy.is_shutdown():
                raise rospy.ServiceException("Super-LIO mode manager is shutting down")
        if operation["error"] is not None:
            raise rospy.ServiceException(str(operation["error"]))
        return operation["response"]

    def _service_start_mapping(self, request):
        return self._call_on_main_thread(self._handle_start_mapping, request)

    def _service_save_map(self, request):
        return self._call_on_main_thread(self._handle_save_map, request)

    def _service_start_localization(self, request):
        return self._call_on_main_thread(self._handle_start_localization, request)

    def _service_stop(self, request):
        return self._call_on_main_thread(self._handle_stop, request)

    def _process_next_operation(self, timeout=0.2):
        try:
            operation = self._operation_queue.get(timeout=timeout)
        except queue.Empty:
            return False
        try:
            operation["response"] = operation["handler"](operation["request"])
        except Exception as exc:
            operation["error"] = exc
            rospy.logerr("Super-LIO main-thread operation failed: %s", exc)
        finally:
            operation["event"].set()
            self._operation_queue.task_done()
        return True

    def spin(self):
        while not rospy.is_shutdown():
            self._process_next_operation()

    def _staging_dir(self):
        return os.path.join(self._map_root, ".staging", "current")

    def _publish_safe(self):
        self._task_enable_pub.publish(Bool(data=False))

    def _initial_pose_callback(self, _message):
        with self._lock:
            if self._state == self.LOCALIZING and self._owned_mode == "localization":
                self._initial_pose_received = True
                self._localization_ready = False
                self._message = "initial pose received; waiting for localization odometry"

    def _localization_odom_callback(self, _message):
        # Set readiness before taking the state lock. Mapping startup holds the
        # lock while waiting, so taking the lock first would prevent the event
        # from ever becoming visible to the startup operation.
        self._odom_event.set()
        with self._lock:
            if (
                self._state == self.LOCALIZING
                and self._owned_mode == "localization"
                and self._initial_pose_received
            ):
                self._localization_ready = True
                self._message = "localization ready"

    def _map_callback(self, _message):
        self._map_event.set()

    @staticmethod
    def _safe_map_id(value):
        text = re.sub(r"[^0-9A-Za-z._-]", "_", str(value or "").strip()).strip("._")
        return text[:96]

    def _bundle_path(self, map_id):
        safe_id = self._safe_map_id(map_id)
        if not safe_id:
            raise RuntimeError("map_id is empty or invalid")
        path = os.path.abspath(os.path.join(self._map_root, safe_id))
        if os.path.commonpath([path, self._map_root]) != self._map_root:
            raise RuntimeError("map bundle escapes configured map root")
        return path

    def _safe_rmtree(self, path):
        target = os.path.realpath(os.path.abspath(path))
        root = os.path.realpath(self._map_root)
        if target == root or os.path.commonpath([target, root]) != root:
            raise RuntimeError("refuse to remove path outside map root: {}".format(target))
        if os.path.isdir(target):
            shutil.rmtree(target)

    @staticmethod
    def _node_names():
        try:
            return set(rosnode.get_node_names())
        except Exception:
            return set()

    def _detect_unowned_conflicts(self):
        conflicts = sorted(self._node_names() & (self._MAPPING_NODES | self._LOCALIZATION_NODES))
        if conflicts:
            self._state = self.ERROR
            self._message = "unowned Super-LIO nodes already running: {}".format(", ".join(conflicts))
            rospy.logerr(self._message)

    def _assert_no_unowned_nodes(self):
        if self._launch_parent is not None:
            return
        conflicts = sorted(self._node_names() & (self._MAPPING_NODES | self._LOCALIZATION_NODES))
        if conflicts:
            raise RuntimeError("unowned Super-LIO nodes already running: {}".format(", ".join(conflicts)))

    def _start_launch(self, launch_file, arguments, owned_mode):
        self._assert_no_unowned_nodes()
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        parent = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, list(arguments))])
        parent.start()
        self._launch_parent = parent
        self._owned_mode = owned_mode

    def _stop_owned_launch(self):
        parent = self._launch_parent
        self._launch_parent = None
        self._owned_mode = ""
        if parent is not None:
            parent.shutdown()
            deadline = time.monotonic() + self._shutdown_timeout
            while time.monotonic() < deadline:
                if not (self._node_names() & (self._MAPPING_NODES | self._LOCALIZATION_NODES)):
                    break
                rospy.sleep(0.1)

    def _wait_service(self, name):
        rospy.wait_for_service(name, timeout=self._startup_timeout)

    def _wait_topic_event(self, event, topic):
        deadline = time.monotonic() + self._startup_timeout
        while not rospy.is_shutdown():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError(
                    "timeout exceeded while waiting for message on topic {}".format(topic)
                )
            if event.wait(min(0.2, remaining)):
                return
        raise RuntimeError("shutdown while waiting for message on topic {}".format(topic))

    def _start_mapping_locked(self):
        self._publish_safe()
        if self._state == self.MAPPING and self._owned_mode == "mapping":
            return "mapping already active"
        self._stop_owned_launch()
        self._assert_no_unowned_nodes()
        self._odom_event.clear()
        self._map_event.clear()
        staging = self._staging_dir()
        if os.path.isdir(staging):
            self._safe_rmtree(staging)
        os.makedirs(staging, exist_ok=True)
        args = [
            "staging_dir:={}".format(staging),
            "loop_closure:={}".format(str(self._loop_closure).lower()),
            "autosave:={}".format(str(self._autosave).lower()),
        ]
        try:
            rospy.loginfo("Starting managed Super-LIO mapping launch")
            self._start_launch(self._mapping_launch, args, "mapping")
            self._wait_service("/lio/get_3dmap")
            self._wait_service("/cloud_to_occupancy_grid/save_map")
            self._wait_service("/cloud_to_occupancy_grid/reset_map")
            rospy.loginfo("Super-LIO mapping services ready; waiting for /lio/odom")
            self._wait_topic_event(self._odom_event, "/lio/odom")
            rospy.loginfo("Super-LIO odometry ready; waiting for /map")
            self._wait_topic_event(self._map_event, "/map")
            rospy.loginfo("Super-LIO mapping topics ready")
        except Exception:
            self._stop_owned_launch()
            raise
        self._state = self.MAPPING
        self._message = "mapping active"
        self._active_map_id = "LIVE_MAP"
        self._bundle_dir = ""
        self._localization_ready = False
        self._initial_pose_received = False
        return self._message

    @staticmethod
    def _validate_pcd(path):
        if not os.path.isfile(path) or os.path.getsize(path) <= 0:
            raise RuntimeError("PCD is missing or empty: {}".format(path))
        points = 0
        with open(path, "rb") as handle:
            for _ in range(80):
                line = handle.readline()
                if not line:
                    break
                text = line.decode("ascii", errors="ignore").strip()
                if text.upper().startswith("POINTS "):
                    try:
                        points = int(text.split(None, 1)[1])
                    except Exception:
                        points = 0
                if text.upper().startswith("DATA "):
                    break
        if points <= 0:
            raise RuntimeError("PCD contains no valid points: {}".format(path))

    @staticmethod
    def _validate_grid(yaml_path, image_path):
        if not os.path.isfile(yaml_path) or os.path.getsize(yaml_path) <= 0:
            raise RuntimeError("map YAML is missing or empty: {}".format(yaml_path))
        if not os.path.isfile(image_path) or os.path.getsize(image_path) <= 0:
            raise RuntimeError("map image is missing or empty: {}".format(image_path))
        with open(yaml_path, "r", encoding="utf-8") as handle:
            yaml_text = handle.read()
        if not re.search(r"(?m)^\s*image\s*:\s*map\.pgm\s*$", yaml_text):
            raise RuntimeError("map YAML must reference map.pgm")

    def _validate_bundle(self, bundle_dir):
        paths = {
            "loc": os.path.join(bundle_dir, "loc_map.pcd"),
            "plan": os.path.join(bundle_dir, "plan_map.pcd"),
            "yaml": os.path.join(bundle_dir, "map.yaml"),
            "image": os.path.join(bundle_dir, "map.pgm"),
        }
        self._validate_pcd(paths["loc"])
        self._validate_pcd(paths["plan"])
        self._validate_grid(paths["yaml"], paths["image"])
        return paths

    def _start_localization_locked(self, map_id):
        self._publish_safe()
        bundle = self._bundle_path(map_id)
        self._validate_bundle(bundle)
        if (
            self._state == self.LOCALIZING
            and self._owned_mode == "localization"
            and self._active_map_id == map_id
        ):
            return bundle, "localization already active"
        self._stop_owned_launch()
        self._odom_event.clear()
        self._map_event.clear()
        args = ["bundle_dir:={}".format(bundle)]
        for key in (
            "base_to_laser_x", "base_to_laser_y", "base_to_laser_z",
            "base_to_laser_roll", "base_to_laser_pitch", "base_to_laser_yaw",
        ):
            args.append("{}:={}".format(key, rospy.get_param("~" + key, 0.0)))
        try:
            self._start_launch(self._localization_launch, args, "localization")
            self._wait_topic_event(self._map_event, "/map")
        except Exception:
            self._stop_owned_launch()
            raise
        self._state = self.LOCALIZING
        self._message = "localization active; initial pose required"
        self._active_map_id = map_id
        self._bundle_dir = bundle
        self._localization_ready = False
        self._initial_pose_received = False
        return bundle, self._message

    def _handle_start_mapping(self, _request):
        with self._lock:
            try:
                message = self._start_mapping_locked()
                return TriggerResponse(success=True, message=message)
            except Exception as exc:
                self._state = self.ERROR
                self._message = str(exc)
                self._publish_safe()
                return TriggerResponse(success=False, message=self._message)

    def _handle_save_map(self, request):
        response = SaveSuperLioMapResponse()
        with self._lock:
            temp_dir = ""
            try:
                if self._state != self.MAPPING or self._owned_mode != "mapping":
                    raise RuntimeError("map save requires active MAPPING state")
                map_id = self._safe_map_id(request.map_id)
                if not map_id:
                    raise RuntimeError("map_id is empty or invalid")
                final_dir = self._bundle_path(map_id)
                temp_dir = os.path.join(self._map_root, ".tmp-{}-{}".format(map_id, os.getpid()))
                if os.path.exists(temp_dir):
                    self._safe_rmtree(temp_dir)
                os.makedirs(temp_dir)
                self._state = self.SAVING
                self._message = "saving map"
                self._publish_safe()

                self._wait_service("/lio/get_3dmap")
                get_map = rospy.ServiceProxy("/lio/get_3dmap", GetMap)
                map_request = GetMapRequest()
                map_request.cmd_id = 1
                map_request.enable_filter = True
                map_request.map_dir = temp_dir
                map_request.map_name = "map"
                map_response = get_map(map_request)
                if not bool(map_response.success):
                    raise RuntimeError("/lio/get_3dmap failed")

                self._wait_service("/cloud_to_occupancy_grid/save_map")
                save_grid = rospy.ServiceProxy("/cloud_to_occupancy_grid/save_map", Trigger)
                grid_response = save_grid()
                if not bool(grid_response.success):
                    raise RuntimeError(grid_response.message or "2D map save failed")
                for filename in ("map.yaml", "map.pgm"):
                    source = os.path.join(self._staging_dir(), filename)
                    if not os.path.isfile(source):
                        raise RuntimeError("staged map file missing: {}".format(source))
                    shutil.copy2(source, os.path.join(temp_dir, filename))

                paths = self._validate_bundle(temp_dir)
                manifest = {
                    "schemaVersion": 2,
                    "mapId": map_id,
                    "mapName": str(request.map_name or ""),
                    "savedAt": int(time.time()),
                    "files": {
                        "localization": "loc_map.pcd",
                        "planning": "plan_map.pcd",
                        "yaml": "map.yaml",
                        "image": "map.pgm",
                    },
                }
                with open(os.path.join(temp_dir, "map_info.json"), "w", encoding="utf-8") as handle:
                    json.dump(manifest, handle, ensure_ascii=False, indent=2)
                if os.path.exists(final_dir):
                    raise RuntimeError("map bundle already exists: {}".format(final_dir))
                os.replace(temp_dir, final_dir)
                temp_dir = ""
                paths = self._validate_bundle(final_dir)

                self._stop_owned_launch()
                localization_started = False
                localization_error = ""
                try:
                    self._start_localization_locked(map_id)
                    localization_started = True
                except Exception as exc:
                    localization_error = str(exc)
                    self._state = self.ERROR
                    self._message = "map saved but localization failed: {}".format(localization_error)

                response.success = True
                response.localization_started = localization_started
                response.message = (
                    "map_saved_and_localization_on"
                    if localization_started
                    else self._message
                )
                response.state = self._state
                response.bundle_dir = final_dir
                response.loc_pcd_path = paths["loc"]
                response.plan_pcd_path = paths["plan"]
                response.yaml_path = paths["yaml"]
                response.image_path = paths["image"]
                return response
            except Exception as exc:
                if temp_dir and os.path.isdir(temp_dir):
                    try:
                        self._safe_rmtree(temp_dir)
                    except Exception as cleanup_exc:
                        rospy.logwarn("Failed to remove temporary map bundle: %s", cleanup_exc)
                # A failed save must leave the running mapper available for retry.
                if self._owned_mode == "mapping" and self._launch_parent is not None:
                    self._state = self.MAPPING
                else:
                    self._state = self.ERROR
                self._message = str(exc)
                self._publish_safe()
                response.success = False
                response.localization_started = False
                response.message = self._message
                response.state = self._state
                return response

    def _handle_start_localization(self, request):
        response = StartSuperLioLocalizationResponse()
        with self._lock:
            try:
                bundle, message = self._start_localization_locked(self._safe_map_id(request.map_id))
                response.success = True
                response.message = message
                response.state = self._state
                response.bundle_dir = bundle
                response.yaml_path = os.path.join(bundle, "map.yaml")
                response.image_path = os.path.join(bundle, "map.pgm")
            except Exception as exc:
                self._state = self.ERROR
                self._message = str(exc)
                self._publish_safe()
                response.success = False
                response.message = self._message
                response.state = self._state
            return response

    def _handle_stop(self, _request):
        with self._lock:
            try:
                self._publish_safe()
                self._stop_owned_launch()
                self._state = self.IDLE
                self._message = "stopped"
                self._active_map_id = ""
                self._bundle_dir = ""
                self._localization_ready = False
                self._initial_pose_received = False
                return TriggerResponse(success=True, message=self._message)
            except Exception as exc:
                self._state = self.ERROR
                self._message = str(exc)
                return TriggerResponse(success=False, message=self._message)

    def _handle_status(self, _request):
        with self._lock:
            return GetSuperLioStatusResponse(
                success=self._state != self.ERROR,
                message=self._message,
                state=self._state,
                active_map_id=self._active_map_id,
                bundle_dir=self._bundle_dir,
                localization_ready=self._localization_ready,
            )

    def shutdown(self):
        with self._lock:
            self._publish_safe()
            self._stop_owned_launch()


def main():
    rospy.init_node("super_lio_mode_manager")
    manager = SuperLioModeManager()
    manager.spin()
