#!/usr/bin/env python3
"""Analytic room regression. Use a separate ROS/Gazebo master, never the APP instance.

python3 check_gpu_lidar.py --write-world /tmp/gpu_room.world
roslaunch grinder_gazebo spawn_x920.launch gui:=false world_name:=/tmp/gpu_room.world
python3 check_gpu_lidar.py --check
"""
import argparse
import math
import time
from pathlib import Path


def write_world(path):
    parts = ['<sdf version="1.6"><world name="gpu_room"><physics type="ode" name="p">'
             '<max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate>'
             '</physics><gravity>0 0 -9.80665</gravity>']
    for name, pose, size in [
        ('gpu_test_floor', '0 0 -0.1', '30 30 0.2'),
        ('east', '8.1 0 5', '0.2 30 10'), ('west', '-6.1 0 5', '0.2 30 10'),
        ('north', '0 7.1 5', '30 0.2 10'), ('south', '0 -5.1 5', '30 0.2 10'),
        ('ceiling', '0 0 10.1', '30 30 0.2'),
    ]:
        geom = '<geometry><box><size>' + size + '</size></box></geometry>'
        parts.append('<model name="' + name + '"><static>true</static><pose>' + pose +
                     ' 0 0 0</pose><link name="l"><collision name="c">' + geom +
                     '</collision><visual name="v">' + geom + '</visual></link></model>')
    parts.append('</world></sdf>')
    Path(path).write_text(''.join(parts))


def check():
    import numpy as np
    import rospy
    from gazebo_msgs.srv import GetLinkState, GetWorldProperties
    from livox_ros_driver2.msg import CustomMsg
    from sensor_msgs.msg import Imu
    from tf.transformations import quaternion_matrix
    rospy.init_node('check_gpu_lidar', anonymous=True)
    rospy.wait_for_service('/gazebo/get_world_properties', timeout=50)
    world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)()
    assert 'gpu_test_floor' in world.model_names, 'This check requires its isolated analytic room'
    rospy.wait_for_message('/livox/lidar', CustomMsg, timeout=50)
    time.sleep(3)  # Let the chassis settle. This also works when /clock is slow.
    clouds = []
    subscriber = rospy.Subscriber('/livox/lidar', CustomMsg, clouds.append, queue_size=30)
    deadline = time.monotonic() + 30
    while len(clouds) < 20 and time.monotonic() < deadline:
        time.sleep(.05)
    subscriber.unregister()
    assert len(clouds) >= 20, 'GPU lidar did not publish enough frames'
    clouds = clouds[-20:]
    imu = rospy.wait_for_message('/livox/imu', Imu, timeout=10)
    assert imu.header.stamp.to_sec() > 0
    pose = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)(
        'x920_grinder::base_link', 'world').link_state.pose
    q = pose.orientation
    rotation = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    origin = np.array([pose.position.x, pose.position.y, pose.position.z]) + rotation @ np.array([.2, 0, .55])
    errors, noise_samples = [], []
    for cloud in clouds:
        assert cloud.header.frame_id == 'laser_link'
        assert cloud.timebase == cloud.header.stamp.to_nsec()
        assert cloud.point_num == len(cloud.points) and cloud.point_num > 6500
        assert all(p.offset_time == 0 and p.tag == 0x10 and p.line < 4 for p in cloud.points)
        points = np.array([[p.x, p.y, p.z] for p in cloud.points])
        assert np.isfinite(points).all()
        angles = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
        assert len(set(np.floor((angles + 180) / 30).astype(int))) >= 12
        elevation = np.degrees(np.arctan2(points[:, 2], np.linalg.norm(points[:, :2], axis=1)))
        assert elevation.min() < -6.9 and elevation.max() > 51.9
        world_points = points @ rotation.T + origin
        mask = (world_points[:, 2] > 1.5) & (world_points[:, 2] < 4)
        residual = np.min(np.abs(world_points[:, :2, None] -
                          np.array([[8, -6], [7, -5]])[None, :, :]), axis=(1, 2))
        errors.extend(residual[mask])
        # One stationary upper forward ray: stochastic range noise must survive.
        index = np.argmin(abs(angles) + abs(elevation - 8.526315789))
        noise_samples.append(np.linalg.norm(points[index]))
    hz = (len(clouds)-1) / (clouds[-1].header.stamp-clouds[0].header.stamp).to_sec()
    median, p99 = np.quantile(errors, [.5, .99])
    noise_std = np.std(noise_samples)
    print('frames=%d points=%d sim_hz=%.3f wall_median_m=%.5f wall_p99_m=%.5f range_std_m=%.5f' %
          (len(clouds), clouds[-1].point_num, hz, median, p99, noise_std), flush=True)
    assert 8 < hz < 12, 'Unexpected scan frequency in simulation time'
    assert median < .02 and p99 < .06, 'Mirrored, shifted or distorted GPU rays'
    assert .002 < noise_std < .03, 'Configured 1 cm range noise missing or excessive'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--write-world')
    group.add_argument('--check', action='store_true')
    args = parser.parse_args()
    if args.write_world:
        write_world(args.write_world)
    else:
        check()
