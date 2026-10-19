#!/usr/bin/env python3
"""Read-only sensor check; --map-cycle exercises managed mapping/save/relocation.

Run only against grinder_sim.launch. No physical driver is accepted.
"""
import argparse
import math
import statistics
import socket
import time

import rosnode
import rospy
import tf2_ros
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_srvs.srv import Trigger
from grinder_scheduler.srv import (
    GetSuperLioStatus, SaveSuperLioMap, SetSuperLioInitialPose,
    SetSuperLioInitialPoseRequest, StartSuperLioLocalization,
)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-cycle", action="store_true")
    args = parser.parse_args()
    rospy.init_node("check_mid360_chain", anonymous=True)
    nodes = rosnode.get_node_names()
    require("/grinder_chassis_sim" in nodes and "/gazebo" in nodes,
            "Run grinder_sim.launch before checking the virtual sensor")
    require(bool(rospy.get_param("/use_sim_time", False)), "Simulation time must be enabled")
    require("/grinder_scheduler" in nodes and "/super_lio_mode_manager" in nodes,
            "Scheduler and managed Super-LIO must be running")
    port = int(rospy.get_param("/grinder_scheduler/sl_linka_port", 8002))
    with socket.create_connection(("127.0.0.1", port), timeout=10):
        pass  # Check the local APP protocol server without sending control commands.
    pubs = dict(rospy.get_master().getSystemState()[2][0])
    require(len(pubs.get("/livox/lidar", [])) == 1 and
            pubs.get("/livox/lidar") == pubs.get("/livox/imu"),
            "LiDAR and IMU must have the same single publisher")
    cloud = rospy.wait_for_message("/livox/lidar", CustomMsg, timeout=30)
    require(cloud.point_num == len(cloud.points) and cloud.point_num > 1000,
            "Expected a nonempty 3D Livox cloud")
    require(cloud.timebase == cloud.header.stamp.to_nsec(), "Point cloud clock mismatch")
    require(all(p.offset_time == 0 for p in cloud.points), "Snapshot has fabricated point times")
    require(max(p.z for p in cloud.points) - min(p.z for p in cloud.points) > 1,
            "Expected a 3D sensor, not a planar scan")
    rospy.sleep(3.0)  # Let the spawned chassis settle onto the floor before sampling.
    samples = []
    first_ray_ranges = []
    lidar_receipts = []
    def sample_range(message):
        if message.points:
            p = message.points[0]
            first_ray_ranges.append(math.sqrt(p.x**2 + p.y**2 + p.z**2))
            lidar_receipts.append(time.monotonic())
    lidar_sub = rospy.Subscriber("/livox/lidar", CustomMsg, sample_range, queue_size=1)
    sub = rospy.Subscriber("/livox/imu", Imu, samples.append, queue_size=1000)
    time.sleep(3)
    sub.unregister()
    lidar_sub.unregister()
    require(len(samples) >= 60, "IMU stream is missing or too slow")
    span = samples[-1].header.stamp.to_sec() - samples[0].header.stamp.to_sec()
    require(span > 0, "IMU timestamps must advance")
    imu_hz = (len(samples) - 1) / span
    mean_gravity = statistics.mean(math.sqrt(
        m.linear_acceleration.x**2 + m.linear_acceleration.y**2 + m.linear_acceleration.z**2
    ) for m in samples)
    require(8.5 < mean_gravity < 11, "IMU must include gravity in m/s^2")
    require(imu_hz > 100, "Expected nominal 200 Hz IMU in simulation time")
    require(len(first_ray_ranges) > 5, "LiDAR stream stopped")
    lidar_wall_hz = (len(lidar_receipts) - 1) / (lidar_receipts[-1] - lidar_receipts[0])
    print("Sensor OK: points={}, IMU={:.1f}Hz(sim), lidar={:.1f}Hz(wall), |accel|={:.3f}, gyro_z_std={:.6f}, first_ray_range_std={:.6f}".format(
        cloud.point_num, imu_hz, lidar_wall_hz, mean_gravity,
        statistics.pstdev(m.angular_velocity.z for m in samples),
        statistics.pstdev(first_ray_ranges)), flush=True)
    if not args.map_cycle:
        return
    rospy.wait_for_service("/super_lio_mode/start_mapping", timeout=30)
    response = rospy.ServiceProxy("/super_lio_mode/start_mapping", Trigger)()
    require(response.success, "Mapping failed: " + response.message)
    print("Managed mapping started; waiting for occupied map cells", flush=True)
    odom = rospy.wait_for_message("/lio/odom", Odometry, timeout=30)
    grid = rospy.wait_for_message("/map", OccupancyGrid, timeout=30)
    scan = rospy.wait_for_message("/scan", LaserScan, timeout=30)
    deadline = time.monotonic() + 30
    while not any(v > 50 for v in grid.data) and time.monotonic() < deadline:
        rospy.sleep(1.0)  # The mapper initially latches an empty grid before its first scan.
        grid = rospy.wait_for_message("/map", OccupancyGrid, timeout=10)
    require(any(v > 50 for v in grid.data), "Mapping produced no occupied cells within 30s")
    require(any(math.isfinite(v) for v in scan.ranges), "LIO produced no navigation scan")
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    transform = buffer.lookup_transform("base_link", "base_laser_link", rospy.Time(0), rospy.Duration(10))
    for axis in "xyz":
        actual = getattr(transform.transform.translation, axis)
        expected = rospy.get_param("/super_lio_mode_manager/base_to_laser_" + axis, 0.0)
        require(abs(actual - expected) < 1e-5, "Mounting TF differs from LIO extrinsic")
    map_id = "sim_check_{}".format(int(time.time()))
    saved = rospy.ServiceProxy("/super_lio_mode/save_map", SaveSuperLioMap)(map_id, map_id)
    require(saved.success and saved.mapping_stopped, "Map save failed: " + saved.message)
    started = rospy.ServiceProxy("/super_lio_mode/start_localization", StartSuperLioLocalization)(map_id)
    require(started.success, "Localization start failed: " + started.message)
    initial = SetSuperLioInitialPoseRequest()
    initial.map_id = map_id
    initial.map_revision = started.asset_revision
    initial.pose.header.stamp = rospy.Time.now()
    initial.pose.header.frame_id = "map"
    initial.pose.pose = odom.pose
    localized = rospy.ServiceProxy("/super_lio_mode/set_initial_pose", SetSuperLioInitialPose)(initial)
    require(localized.success, "Initial pose failed: " + localized.message)
    get_status = rospy.ServiceProxy("/super_lio_mode/get_status", GetSuperLioStatus)
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        state = get_status()
        if state.localization_ready:
            print("Map cycle OK: mapping -> save -> localization READY; " + saved.bundle_dir, flush=True)
            return
        time.sleep(0.2)
    raise RuntimeError("Localization did not reach READY: " + state.message)


if __name__ == "__main__":
    main()
