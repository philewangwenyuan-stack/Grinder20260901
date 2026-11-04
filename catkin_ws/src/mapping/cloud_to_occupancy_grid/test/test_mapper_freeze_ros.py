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
                        "__name:=freeze_mapper", "_map_width:=4", "_map_height:=4",
                        "_origin_x:=-2", "_origin_y:=-2", "_sensor_frame:=map",
                        "_cloud_topic:=/freeze_fixture/cloud", "_map_topic:=/freeze_fixture/map",
                        "_tablet_map_topic:=", "_reprojection_enabled:=false",
                        "_publish_cropped_map:=false", "_map_directory:=" + directory,
                    ], stdout=log, stderr=log, start_new_session=True))
                    rospy.wait_for_service("/freeze_mapper/freeze_map", timeout=20)
                    freeze = rospy.ServiceProxy("/freeze_mapper/freeze_map", Trigger)
                    publish = rospy.Publisher("/freeze_fixture/cloud", PointCloud2, queue_size=1, latch=True)
                    deadline = time.monotonic() + 5
                    while publish.get_num_connections() == 0:
                        if time.monotonic() > deadline: self.fail("mapper did not subscribe")
                        time.sleep(.05)
                    stamp = rospy.Time.now()
                    rospy.set_param("/freeze_mapper/freeze_target_stamp_ns", str(stamp.to_nsec()))
                    self.assertFalse(freeze().success)  # Target scan has not arrived yet.
                    publish.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id="map"), [(1., 0., .3)]))
                    deadline = time.monotonic() + 5
                    while not freeze().success:
                        if time.monotonic() > deadline: self.fail("could not freeze the final scan")
                        time.sleep(.05)
                    before = rospy.wait_for_message("/freeze_fixture/map", OccupancyGrid, timeout=5)
                    self.assertIn(100, before.data)
                    publish.publish(point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.now(), frame_id="map"), [(-1., 0., .3)]))
                    time.sleep(.2)
                    self.assertFalse(rospy.ServiceProxy("/freeze_mapper/reset_map", Trigger)().success)
                    self.assertTrue(rospy.ServiceProxy("/freeze_mapper/save_map", Trigger)().success)
                    after = rospy.wait_for_message("/freeze_fixture/map", OccupancyGrid, timeout=5)
                    self.assertEqual(before.data, after.data)
                    self.assertTrue(os.path.getsize(os.path.join(directory, "grinder_map.pgm")) > 0)
                finally:
                    for process in reversed(processes):
                        if process.poll() is None:
                            os.killpg(process.pid, signal.SIGINT)
                            try: process.wait(timeout=10)
                            except subprocess.TimeoutExpired:
                                os.killpg(process.pid, signal.SIGKILL)
                                process.wait(timeout=5)


if __name__ == "__main__": unittest.main()
