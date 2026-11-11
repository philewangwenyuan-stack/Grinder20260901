"""Fleet isolation and bridge watchdog regressions; no ROS/hardware required."""
import copy
from pathlib import Path
import queue
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from grinder_gazebo.fleet_config import CommandDeadline, isolate_urdf, sensor_routes, validate_fleet
from grinder_gazebo.fleet_bridge import put_latest, serialize
from grinder_gazebo.fleet_runner import prepare_robot

PACKAGE = Path(__file__).resolve().parents[1]
URDF = '''<robot name="old"><link name="base_link"/><gazebo>
<plugin filename="libgazebo_ros_diff_drive.so"><robotNamespace>/</robotNamespace>
<commandTopic>/grinder/sim/cmd_vel</commandTopic><odometryTopic>/grinder/sim/ground_truth/odom</odometryTopic>
<robotBaseFrame>base_link</robotBaseFrame></plugin>
<sensor type="gpu_ray"><ray><range><max>45</max></range></ray>
<plugin filename="libgrinder_mid360_sensor.so"/></sensor>
<sensor type="imu"><plugin filename="libgazebo_ros_imu_sensor.so"><robotNamespace>/</robotNamespace>
<topicName>/livox/imu</topicName><frameName>mid360_imu_link</frameName></plugin></sensor>
<sensor type="camera"><plugin filename="libgazebo_ros_camera.so"><imageTopicName>/grinder/sim/left/image_raw</imageTopicName>
<cameraInfoTopicName>/grinder/sim/left/camera_info</cameraInfoTopicName></plugin></sensor></gazebo></robot>'''


class FleetTest(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((PACKAGE / "config/fleet.yaml").read_text())

    def test_three_robots_have_unique_ports_and_spawn_positions(self):
        fleet = validate_fleet(self.config, 3, [-84, 50, .2])
        self.assertEqual([-84., 53., .2], fleet["robots"][1]["spawn"])
        self.assertEqual([8102, 8202, 8302], [r["app_port"] for r in fleet["robots"]])
        self.assertNotIn("spawn", self.config["robots"][0])

    def test_rejects_ambiguous_ports_names_overlaps_and_nonfinite_poses(self):
        for change in (dict(master_port=11411), dict(control_port=8003),
                       dict(name="../other"), dict(name="robot1"),
                       dict(spawn=[0, 0, 0]), dict(spawn=[float("nan"), 3, 0])):
            config = copy.deepcopy(self.config)
            config["robots"][1].update(change)
            with self.assertRaises(ValueError):
                validate_fleet(config, 2)

    def test_urdf_isolates_plugins_without_changing_frames_or_sensor_geometry(self):
        first = ET.fromstring(isolate_urdf(URDF, "robot1"))
        second = ET.fromstring(isolate_urdf(URDF, "robot2"))
        self.assertEqual("/robot1/livox/lidar", first.find(".//topic_name").text)
        self.assertEqual("/robot2/livox/imu", second.find(".//topicName").text)
        self.assertEqual("/robot1/grinder/sim/cmd_vel", first.find(".//commandTopic").text)
        self.assertEqual("base_link", first.find(".//robotBaseFrame").text)
        self.assertEqual("45", first.find(".//ray/range/max").text)
        self.assertFalse(first.findall(".//sensor[@type='camera']"))
        self.assertIn('imageTopicName>/robot2/', isolate_urdf(URDF, "robot2", True))

    def test_bridge_exposes_only_sensor_routes_not_tf_map_or_chassis(self):
        routes = sensor_routes("robot2")
        for source, destination, _, _ in routes:
            self.assertTrue(source == "/clock" or source.startswith("/robot2/"))
            self.assertNotIn(destination, ("/tf", "/tf_static", "/map", "/cmd_vel", "/chassis/task_enable"))
        self.assertEqual(1, sum(r[1] == "/livox/imu" for r in routes))

    def test_stale_commands_cannot_keep_motion_alive_when_clock_is_paused(self):
        watchdog = CommandDeadline(timeout=.5, max_age=.25)
        self.assertTrue(watchdog.expired(10))
        self.assertTrue(watchdog.accept(10, 10.1))
        self.assertFalse(watchdog.expired(10.4))
        self.assertFalse(watchdog.accept(9, 10.6))
        self.assertTrue(watchdog.expired(10.6))
        self.assertFalse(watchdog.accept(12, 11))

    def test_bounded_queue_replaces_old_command(self):
        channel = queue.Queue(maxsize=1)
        put_latest(channel, "old")
        put_latest(channel, "new")
        self.assertEqual("new", channel.get_nowait())

    def test_wire_bridge_accepts_rospy_typed_clock_and_raw_lidar(self):
        class TypedClock:
            def serialize(self, buffer):
                buffer.write(b"clock")
        class RawLidar:
            _buff = b"original-lidar-stamps-and-points"
            def serialize(self, buffer):
                raise AssertionError("Do not deserialize/reserialize raw lidar")
        self.assertEqual(b"clock", serialize(TypedClock()))
        self.assertEqual(RawLidar._buff, serialize(RawLidar()))

    def test_writable_paths_are_per_robot_and_cannot_overwrite_source(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory) / "catkin_ws"
            nav = workspace / "src/2-dnavigation-package/2dnavigation/teb_local_planner_tutorials/cfg/diff_drive"
            nav.mkdir(parents=True)
            for filename in ("costmap_common_params.yaml", "rpp_local_planner_params.yaml"):
                (nav / filename).write_text("original: true\n")
            robots = validate_fleet(self.config, 2)["robots"]
            for i, robot in enumerate(robots):
                robot["rtsp_port"] = 8654 + i * 100
                root, maps = prepare_robot(workspace, PACKAGE, robot, False, URDF)
                config = yaml.safe_load((root / "runtime.yaml").read_text())
                for key, value in config.items():
                    if isinstance(value, str):
                        self.assertTrue(Path(value).relative_to(root).parts, (key, value))
                self.assertEqual(robot["control_port"], config["sl_linka_control_port"])
                self.assertEqual(robot["name"], maps.name)
                (root / "rpp.yaml").write_text("custom: true\n")
                prepare_robot(workspace, PACKAGE, robot, False, URDF)
                self.assertIn("custom", (root / "rpp.yaml").read_text())
            self.assertEqual("original: true\n", (nav / "rpp_local_planner_params.yaml").read_text())


if __name__ == "__main__":
    unittest.main()
