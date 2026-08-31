#!/usr/bin/env python3
"""Expose Gazebo data through the subset of slamware_ros_sdk used by Grinder."""

import json
import math
import os
import threading
import time

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import LoadMap, LoadMapResponse
from sensor_msgs.msg import Image

from slamware_ros_sdk.msg import (
    ClearMapRequest,
    RelocalizationStatus,
    SetMapLocalizationRequest,
    SetMapUpdateRequest,
    SyncMapRequest,
    SystemStatus,
)
from slamware_ros_sdk.srv import (
    RelocalizationRequest,
    RelocalizationRequestResponse,
    SyncGetStcm,
    SyncGetStcmResponse,
    SyncSetStcm,
    SyncSetStcmResponse,
)


class SlamwareSimBridge:
    def __init__(self):
        self._lock = threading.RLock()
        self._namespace = str(rospy.get_param("~slamware_namespace", "/slamware_ros_sdk_server_node")).rstrip("/")
        self._sim_odom_topic = rospy.get_param("~sim_odom_topic", "/odom")
        self._left_input_topic = rospy.get_param("~left_image_input", "/grinder/sim/left/image_raw")
        self._right_input_topic = rospy.get_param("~right_image_input", "/grinder/sim/right/image_raw")
        self._map_frame = rospy.get_param("~map_frame", "map")
        self._odom_frame = rospy.get_param("~odom_frame", "odom")
        self._map_resolution = max(0.01, float(rospy.get_param("~map_resolution", 0.05)))
        self._world_width = max(2.0, float(rospy.get_param("~world_width_m", 12.0)))
        self._world_height = max(2.0, float(rospy.get_param("~world_height_m", 10.0)))
        self._column_size = max(0.1, float(rospy.get_param("~column_size_m", 0.9)))
        self._column_spacing = max(self._column_size, float(rospy.get_param("~column_spacing_m", 6.0)))
        self._column_edge_margin = max(
            self._column_size * 0.5,
            float(rospy.get_param("~column_edge_margin_m", 4.0)),
        )

        self._mapping_enabled = False
        self._localization_enabled = True
        self._last_odom = None
        self._map = self._build_factory_map()

        self._map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self._slam_map_pub = rospy.Publisher(self._namespace + "/map", OccupancyGrid, queue_size=1, latch=True)
        self._odom_pub = rospy.Publisher(self._namespace + "/odom", Odometry, queue_size=20)
        self._left_pub = rospy.Publisher(self._namespace + "/left_image_raw", Image, queue_size=1)
        self._right_pub = rospy.Publisher(self._namespace + "/right_image_raw", Image, queue_size=1)
        self._depth_color_pub = rospy.Publisher(
            self._namespace + "/depth_image_colorized", Image, queue_size=1
        )
        self._system_status_pub = rospy.Publisher(
            self._namespace + "/system_status", SystemStatus, queue_size=5, latch=True
        )
        self._relocalization_status_pub = rospy.Publisher(
            self._namespace + "/relocalization_status", RelocalizationStatus, queue_size=5, latch=True
        )

        rospy.Subscriber(self._sim_odom_topic, Odometry, self._on_odom, queue_size=20)
        rospy.Subscriber(self._left_input_topic, Image, self._on_left_image, queue_size=1)
        rospy.Subscriber(self._right_input_topic, Image, self._on_right_image, queue_size=1)
        rospy.Subscriber(
            self._namespace + "/set_map_update", SetMapUpdateRequest, self._on_set_map_update, queue_size=5
        )
        rospy.Subscriber(
            self._namespace + "/set_map_localization",
            SetMapLocalizationRequest,
            self._on_set_map_localization,
            queue_size=5,
        )
        rospy.Subscriber(self._namespace + "/clear_map", ClearMapRequest, self._on_clear_map, queue_size=5)
        rospy.Subscriber(self._namespace + "/sync_map", SyncMapRequest, self._on_sync_map, queue_size=5)

        rospy.Service(self._namespace + "/sync_get_stcm", SyncGetStcm, self._handle_sync_get_stcm)
        rospy.Service(self._namespace + "/sync_set_stcm", SyncSetStcm, self._handle_sync_set_stcm)
        rospy.Service(self._namespace + "/relocalization", RelocalizationRequest, self._handle_relocalization)
        rospy.Service("/change_map", LoadMap, self._handle_change_map)

        self._static_tf = tf2_ros.StaticTransformBroadcaster()
        self._status_timer = rospy.Timer(rospy.Duration(1.0), self._status_tick)
        self._map_timer = rospy.Timer(rospy.Duration(2.0), self._map_tick)
        rospy.Timer(rospy.Duration(0.2), self._publish_initial_state, oneshot=True)
        rospy.loginfo(
            "slamware simulator bridge ready: namespace=%s map=%dx%d@%.3fm",
            self._namespace,
            self._map.info.width,
            self._map.info.height,
            self._map.info.resolution,
        )

    @staticmethod
    def _set_rect(data, width, height, resolution, origin_x, origin_y, cx, cy, sx, sy, value=100):
        min_col = max(0, int(math.floor((cx - sx * 0.5 - origin_x) / resolution)))
        max_col = min(width - 1, int(math.ceil((cx + sx * 0.5 - origin_x) / resolution)))
        min_row = max(0, int(math.floor((cy - sy * 0.5 - origin_y) / resolution)))
        max_row = min(height - 1, int(math.ceil((cy + sy * 0.5 - origin_y) / resolution)))
        for row in range(min_row, max_row + 1):
            offset = row * width
            for col in range(min_col, max_col + 1):
                data[offset + col] = int(value)

    def _build_factory_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = self._map_frame
        grid.info.resolution = self._map_resolution
        grid.info.width = int(round(self._world_width / self._map_resolution))
        grid.info.height = int(round(self._world_height / self._map_resolution))
        grid.info.origin.position.x = -self._world_width * 0.5
        grid.info.origin.position.y = -self._world_height * 0.5
        grid.info.origin.orientation.w = 1.0
        data = [0] * (grid.info.width * grid.info.height)

        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        # Match factory_floor.world: a 50 m square, 0.2 m boundary walls,
        # and a regular 0.9 m square column grid at 6 m center spacing.
        half_width = self._world_width * 0.5
        half_height = self._world_height * 0.5
        wall_thickness = 0.2
        self._set_rect(
            data, grid.info.width, grid.info.height, self._map_resolution,
            ox, oy, 0, half_height, self._world_width, wall_thickness
        )
        self._set_rect(
            data, grid.info.width, grid.info.height, self._map_resolution,
            ox, oy, 0, -half_height, self._world_width, wall_thickness
        )
        self._set_rect(
            data, grid.info.width, grid.info.height, self._map_resolution,
            ox, oy, half_width, 0, wall_thickness, self._world_height
        )
        self._set_rect(
            data, grid.info.width, grid.info.height, self._map_resolution,
            ox, oy, -half_width, 0, wall_thickness, self._world_height
        )

        x_start = -half_width + self._column_edge_margin
        y_start = -half_height + self._column_edge_margin
        x_end = half_width - self._column_edge_margin
        y_end = half_height - self._column_edge_margin
        x = x_start
        while x <= x_end + 1e-9:
            y = y_start
            while y <= y_end + 1e-9:
                self._set_rect(
                    data, grid.info.width, grid.info.height, self._map_resolution,
                    ox, oy, x, y, self._column_size, self._column_size
                )
                y += self._column_spacing
            x += self._column_spacing
        grid.data = data
        return grid

    def _stamp_map(self):
        now = rospy.Time.now()
        self._map.header.stamp = now
        self._map.info.map_load_time = now
        return self._map

    def _publish_map(self):
        message = self._stamp_map()
        self._map_pub.publish(message)
        self._slam_map_pub.publish(message)

    def _publish_map_to_odom_tf(self):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._odom_frame
        transform.transform.rotation.w = 1.0
        self._static_tf.sendTransform(transform)

    def _publish_initial_state(self, _event):
        self._publish_map_to_odom_tf()
        self._publish_map()
        self._publish_relocalization_status("RelocalizationSucceed")
        self._publish_system_status()

    def _on_odom(self, message):
        if rospy.is_shutdown():
            return
        with self._lock:
            self._last_odom = message
        output = Odometry()
        output.header = message.header
        output.header.frame_id = self._odom_frame
        output.child_frame_id = message.child_frame_id or "base_footprint"
        output.pose = message.pose
        output.twist = message.twist
        self._odom_pub.publish(output)

    def _on_left_image(self, message):
        if rospy.is_shutdown():
            return
        self._left_pub.publish(message)
        # The scheduler consumes a colorized depth image. Reusing the simulated
        # left image preserves the complete APP video path without fabricating
        # metric depth values.
        self._depth_color_pub.publish(message)

    def _on_right_image(self, message):
        if rospy.is_shutdown():
            return
        self._right_pub.publish(message)

    def _on_set_map_update(self, message):
        with self._lock:
            self._mapping_enabled = bool(message.enabled)
            if self._mapping_enabled:
                self._localization_enabled = False
        self._publish_system_status()

    def _on_set_map_localization(self, message):
        with self._lock:
            self._localization_enabled = bool(message.enabled)
            if self._localization_enabled:
                self._mapping_enabled = False
        self._publish_system_status()

    def _on_clear_map(self, _message):
        # The simulated world is static. Treat clear as a deterministic reset
        # rather than publishing an empty map that would invalidate navigation.
        with self._lock:
            self._map = self._build_factory_map()
        self._publish_map()
        rospy.loginfo("Simulated radar map reset to factory world")

    def _on_sync_map(self, _message):
        self._publish_map()

    def _status_text(self):
        with self._lock:
            if self._mapping_enabled:
                mode = "mapping"
            elif self._localization_enabled:
                mode = "localization"
            else:
                mode = "idle"
        return "SimulatorReady:{}".format(mode)

    def _publish_system_status(self):
        message = SystemStatus()
        message.timestamp_ns = int(rospy.Time.now().to_nsec())
        message.status = self._status_text()
        self._system_status_pub.publish(message)

    def _publish_relocalization_status(self, status):
        message = RelocalizationStatus()
        message.timestamp_ns = int(rospy.Time.now().to_nsec())
        message.status = str(status)
        self._relocalization_status_pub.publish(message)

    def _status_tick(self, _event):
        self._publish_system_status()

    def _map_tick(self, _event):
        self._publish_map()

    def _handle_sync_get_stcm(self, request):
        target = os.path.abspath(str(request.mapfile or "").strip())
        if not target.lower().endswith(".stcm"):
            return SyncGetStcmResponse(success=False, message="sim_stcm_path_must_end_with_stcm")
        try:
            parent = os.path.dirname(target)
            if parent and not os.path.isdir(parent):
                os.makedirs(parent)
            payload = {
                "format": "GRINDER_SIM_STCM_V1",
                "created_unix_s": time.time(),
                "map": {
                    "frame": self._map_frame,
                    "resolution": float(self._map.info.resolution),
                    "width": int(self._map.info.width),
                    "height": int(self._map.info.height),
                    "origin": [
                        float(self._map.info.origin.position.x),
                        float(self._map.info.origin.position.y),
                    ],
                },
            }
            temporary = target + ".tmp"
            with open(temporary, "w", encoding="utf-8") as handle:
                handle.write("GRINDER_SIM_STCM_V1\n")
                json.dump(payload, handle, ensure_ascii=False, sort_keys=True)
                handle.write("\n")
            os.replace(temporary, target)
            return SyncGetStcmResponse(success=True, message="sim_stcm_saved")
        except Exception as exc:
            rospy.logerr("Failed to save simulated STCM %s: %s", target, exc)
            return SyncGetStcmResponse(success=False, message=str(exc))

    def _handle_sync_set_stcm(self, request):
        target = os.path.abspath(str(request.mapfile or "").strip())
        if not target.lower().endswith(".stcm") or not os.path.isfile(target):
            return SyncSetStcmResponse(success=False, message="sim_stcm_not_found")
        self._publish_map()
        return SyncSetStcmResponse(success=True, message="sim_stcm_loaded")

    def _finish_relocalization(self, _event):
        self._publish_relocalization_status("RelocalizationSucceed")

    def _handle_relocalization(self, _request):
        self._publish_relocalization_status("RelocalizationRunning")
        rospy.Timer(rospy.Duration(0.5), self._finish_relocalization, oneshot=True)
        return RelocalizationRequestResponse(success=True)

    def _handle_change_map(self, _request):
        self._publish_map()
        return LoadMapResponse(map=self._map, result=0)


def main():
    rospy.init_node("slamware_sim_bridge")
    SlamwareSimBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
