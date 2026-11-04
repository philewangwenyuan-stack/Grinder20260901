#!/usr/bin/env python3
"""Standalone ROS integration test. Starts only a private master and synthetic mapper."""
import os
import signal
import socket
import subprocess
import tempfile
import time
import unittest


class MapperFreezeTest(unittest.TestCase):
    def test_final_frame_is_frozen_and_cannot_be_reset_or_overwritten(self):
        import rospy
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs import point_cloud2
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import Header
        from std_srvs.srv import Trigger

        port = 11379
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", port))  # Refuse to reuse another test/live master.
        os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:{}".format(port)
        os.environ["ROS_IP"] = "127.0.0.1"
        os.environ.pop("ROS_HOSTNAME", None)
        processes = []
        with tempfile.TemporaryDirectory(prefix="mapper-freeze-") as directory:
            with open(os.path.join(directory, "ros.log"), "w") as log:
                try:
                    processes.append(subprocess.Popen(["roscore", "-p", str(port)], stdout=log,
                                                      stderr=log, start_new_session=True))
                    deadline = time.monotonic() + 20
                    while True:
                        try:
                            with socket.create_connection(("127.0.0.1", port), timeout=.2): break
                        except OSError:
                            if time.monotonic() >= deadline: raise
                            time.sleep(.1)
                    rospy.init_node("freeze_fixture", anonymous=True, disable_signals=True)
                    processes.append(subprocess.Popen([
                        "rosrun", "cloud_to_occupancy_grid", "cloud_to_occupancy_grid_node",
                        "__name:=freeze_mapper", "_map_width:=500", "_map_height:=500",
                        "_origin_x:=-60", "_origin_y:=-60", "_sensor_frame:=map",
                        "_cloud_topic:=/freeze_fixture/cloud", "_map_topic:=/freeze_fixture/map",
                        "_tablet_map_topic:=", "_reprojection_enabled:=false",
                        "_publish_cropped_map:=true", "_save_cropped_map:=true", "_map_directory:=" + directory,
                    ], stdout=log, stderr=log, start_new_session=True))
                    rospy.wait_for_service("/freeze_mapper/freeze_map", timeout=20)
                    freeze = rospy.ServiceProxy("/freeze_mapper/freeze_map", Trigger)
                    publish = rospy.Publisher("/freeze_fixture/cloud", PointCloud2, queue_size=1, latch=True)
                    deadline = time.monotonic() + 5
                    while publish.get_num_connections() == 0:
                        if time.monotonic() > deadline: self.fail("mapper did not subscribe")
                        time.sleep(.05)
                    # A 500m capacity must start with no tiles and a 1-cell unknown map.
                    from std_msgs.msg import String
                    empty = rospy.wait_for_message("/freeze_fixture/map", OccupancyGrid, timeout=5)
                    self.assertEqual((empty.info.width, empty.info.height, list(empty.data)), (1, 1, [-1]))
                    status = rospy.wait_for_message("/freeze_mapper/storage_status", String, timeout=5)
                    self.assertIn("tile_bytes=0 ", status.data)
                    with open("/proc/{}/status".format(processes[-1].pid)) as proc:
                        rss_kib = next(int(line.split()[1]) for line in proc if line.startswith("VmRSS:"))
                    print("500m empty map RSS: {} KiB".format(rss_kib), flush=True)
                    self.assertLess(rss_kib, 200 * 1024)  # Old dense payload alone was 477 MiB.
                    # Observe across tile boundaries on both sides of the world origin.
                    publish.publish(point_cloud2.create_cloud_xyz32(
                        Header(stamp=rospy.Time.now(), frame_id="map"), [(5., 0., .3), (-5., 0., .3)]))
                    deadline = time.monotonic() + 5
                    while True:
                        expanded = rospy.wait_for_message("/freeze_fixture/map", OccupancyGrid, timeout=5)
                        if 100 in expanded.data: break
                        if time.monotonic() > deadline: self.fail("scan not integrated")
                    self.assertAlmostEqual(expanded.info.origin.position.x, -6., places=5)
                    self.assertLess(expanded.info.width * expanded.info.height, 10000)
                    self.assertTrue(rospy.ServiceProxy("/freeze_mapper/reset_map", Trigger)().success)
                    time.sleep(1.2)
                    cleared = rospy.wait_for_message("/freeze_fixture/map", OccupancyGrid, timeout=5)
                    self.assertEqual(len(cleared.data), 1)
                    status = rospy.wait_for_message("/freeze_mapper/storage_status", String, timeout=5)
                    self.assertIn("tile_bytes=0 ", status.data)
                    stamp = rospy.Time.now()
                    rospy.set_param("/freeze_mapper/freeze_target_stamp_ns", str(stamp.to_nsec()))
                    self.assertFalse(freeze().success)  # Target scan has not arrived yet.
                    publish.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id="map"), [(1., 0., .3)]))
                    deadline = time.monotonic() + 5
                    while not freeze().success:
                        if time.monotonic() > deadline: self.fail("could not freeze the final scan")
                        time.sleep(.05)
                    deadline = time.monotonic() + 5
                    while True:
                        before = rospy.wait_for_message("/freeze_fixture/map", OccupancyGrid, timeout=5)
                        if 100 in before.data: break
                        if time.monotonic() > deadline: self.fail("frozen map not published")
                        time.sleep(.1)
                    publish.publish(point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.now(), frame_id="map"), [(-1., 0., .3)]))
                    time.sleep(.2)
                    self.assertFalse(rospy.ServiceProxy("/freeze_mapper/reset_map", Trigger)().success)
                    self.assertTrue(rospy.ServiceProxy("/freeze_mapper/save_map", Trigger)().success)
                    after = rospy.wait_for_message("/freeze_fixture/map", OccupancyGrid, timeout=5)
                    self.assertEqual(before.data, after.data)
                    pgm_path = os.path.join(directory, "grinder_map.pgm")
                    with open(pgm_path, "rb") as pgm:
                        self.assertEqual(pgm.readline(), b"P5\n")
                        pgm.readline()
                        self.assertEqual(tuple(map(int, pgm.readline().split())),
                                         (after.info.width, after.info.height))
                    # A separate mapper with a tiny snapshot cap rejects publication/save/freeze.
                    processes.append(subprocess.Popen([
                        "rosrun", "cloud_to_occupancy_grid", "cloud_to_occupancy_grid_node",
                        "__name:=limited_mapper", "_sensor_frame:=map", "_reprojection_enabled:=false",
                        "_cloud_topic:=/limit_fixture/cloud", "_map_topic:=/limit_fixture/map",
                        "_tablet_map_topic:=", "_autosave:=false", "_max_snapshot_cells:=10",
                        "_map_directory:=" + directory + "/limited",
                    ], stdout=log, stderr=log, start_new_session=True))
                    rospy.wait_for_service("/limited_mapper/freeze_map", timeout=20)
                    limited_publish = rospy.Publisher("/limit_fixture/cloud", PointCloud2, queue_size=1, latch=True)
                    deadline = time.monotonic() + 5
                    while limited_publish.get_num_connections() == 0:
                        if time.monotonic() > deadline: self.fail("limited mapper not subscribed")
                        time.sleep(.05)
                    limited_publish.publish(point_cloud2.create_cloud_xyz32(
                        Header(stamp=rospy.Time.now(), frame_id="map"), [(1., 0., .3)]))
                    time.sleep(.5)
                    result = rospy.ServiceProxy("/limited_mapper/freeze_map", Trigger)()
                    self.assertFalse(result.success)
                    self.assertIn("max_snapshot_cells", result.message)
                    self.assertFalse(rospy.ServiceProxy("/limited_mapper/save_map", Trigger)().success)
                    self.assertFalse(os.path.exists(os.path.join(directory, "limited", "grinder_map.pgm")))
                    # Exhaust a 1 MiB tile budget with a dense fan, then verify fail-closed and reset.
                    processes.append(subprocess.Popen([
                        "rosrun", "cloud_to_occupancy_grid", "cloud_to_occupancy_grid_node",
                        "__name:=budget_mapper", "_sensor_frame:=map", "_reprojection_enabled:=false",
                        "_cloud_topic:=/budget_fixture/cloud", "_map_topic:=/budget_fixture/map",
                        "_tablet_map_topic:=", "_autosave:=false", "_max_grid_memory_mb:=1",
                        "_map_directory:=" + directory + "/budget",
                    ], stdout=log, stderr=log, start_new_session=True))
                    rospy.wait_for_service("/budget_mapper/freeze_map", timeout=20)
                    budget_publish = rospy.Publisher("/budget_fixture/cloud", PointCloud2, queue_size=1, latch=True)
                    deadline = time.monotonic() + 5
                    while budget_publish.get_num_connections() == 0:
                        if time.monotonic() > deadline: self.fail("budget mapper not subscribed")
                        time.sleep(.05)
                    import math
                    points = [(29. * math.cos(a * math.pi / 360),
                               29. * math.sin(a * math.pi / 360), .3) for a in range(720)]
                    budget_publish.publish(point_cloud2.create_cloud_xyz32(
                        Header(stamp=rospy.Time.now(), frame_id="map"), points))
                    deadline = time.monotonic() + 5
                    while True:
                        status = rospy.wait_for_message("/budget_mapper/storage_status", String, timeout=5)
                        if status.data.startswith("ERROR "): break
                        if time.monotonic() > deadline: self.fail("tile cap was not enforced")
                    tile_bytes = int(status.data.split("tile_bytes=")[1].split()[0])
                    self.assertLessEqual(tile_bytes, 1024 * 1024)
                    self.assertFalse(rospy.ServiceProxy("/budget_mapper/freeze_map", Trigger)().success)
                    self.assertFalse(rospy.ServiceProxy("/budget_mapper/save_map", Trigger)().success)
                    self.assertTrue(rospy.ServiceProxy("/budget_mapper/reset_map", Trigger)().success)
                    time.sleep(1.2)
                    status = rospy.wait_for_message("/budget_mapper/storage_status", String, timeout=5)
                    self.assertTrue(status.data.startswith("OK "))
                    self.assertIn("tile_bytes=0 ", status.data)
                    # Loop replay uses the same tiles and atomically replaces observed bounds.
                    from super_lio_loop.srv import GetKeyframe, GetKeyframeResponse
                    from std_msgs.msg import UInt64
                    def keyframe(request):
                        response = GetKeyframeResponse()
                        response.success = True
                        response.total = 1
                        response.pose.orientation.w = 1.
                        response.pose.position.x = 10.
                        response.cloud = point_cloud2.create_cloud_xyz32(
                            Header(stamp=rospy.Time.now(), frame_id="sensor"), [(1., 0., .3)])
                        return response
                    keyframe_service = rospy.Service("/replay_fixture/keyframe", GetKeyframe, keyframe)
                    processes.append(subprocess.Popen([
                        "rosrun", "cloud_to_occupancy_grid", "cloud_to_occupancy_grid_node",
                        "__name:=replay_mapper", "_sensor_frame:=map", "_reprojection_enabled:=true",
                        "_loop_revision_topic:=/replay_fixture/revision", "_keyframe_service:=/replay_fixture/keyframe",
                        "_cloud_topic:=/replay_fixture/cloud", "_map_topic:=/replay_fixture/map",
                        "_tablet_map_topic:=", "_autosave:=false",
                    ], stdout=log, stderr=log, start_new_session=True))
                    rospy.wait_for_service("/replay_mapper/freeze_map", timeout=20)
                    revision = rospy.Publisher("/replay_fixture/revision", UInt64, queue_size=1, latch=True)
                    revision.publish(UInt64(data=1))
                    deadline = time.monotonic() + 10
                    while True:
                        replay = rospy.wait_for_message("/replay_fixture/map", OccupancyGrid, timeout=5)
                        if 100 in replay.data: break
                        if time.monotonic() > deadline: self.fail("reprojection did not complete")
                        time.sleep(.1)
                    self.assertAlmostEqual(replay.info.origin.position.x, 9., places=5)
                    self.assertIn(0, replay.data)
                    keyframe_service.shutdown()

                finally:
                    for process in reversed(processes):
                        if process.poll() is None:
                            os.killpg(process.pid, signal.SIGINT)
                            try: process.wait(timeout=10)
                            except subprocess.TimeoutExpired:
                                os.killpg(process.pid, signal.SIGKILL)
                                process.wait(timeout=5)


if __name__ == "__main__": unittest.main()
