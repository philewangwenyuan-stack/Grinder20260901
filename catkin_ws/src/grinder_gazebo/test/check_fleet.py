#!/usr/bin/env python3
"""Check a running fleet. --map-cycle/--drive affect only an idle test fleet.

Default: sensor/master/storage checks. --map-cycle starts/stops independent maps.
--drive opens real SL-LinkA paired sessions and briefly drives one car at a time.
"""
import argparse
import math
import multiprocessing as mp
import os
from pathlib import Path
import json
import socket
import sys
import time


def robot_check(robot, map_cycle, results):
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:{}/".format(robot["master_port"])
    import rospy
    import rosnode
    from livox_ros_driver2.msg import CustomMsg
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import OccupancyGrid
    from std_srvs.srv import Trigger
    from grinder_scheduler.srv import GetSuperLioStatus
    rospy.init_node("check_fleet_robot", anonymous=True)
    started = False
    try:
        assert "/grinder_chassis_sim" in rosnode.get_node_names(), "Not a simulated chassis"
        assert "/gazebo" not in rosnode.get_node_names(), "Robot stack shares the world master"
        rospy.wait_for_message("/livox/imu", Imu, timeout=20)
        cloud = rospy.wait_for_message("/livox/lidar", CustomMsg, timeout=20)
        assert cloud.point_num > 100
        pubs = dict(rospy.get_master().getSystemState()[2][0])
        assert pubs["/livox/lidar"] == ["/fleet_{}_robot_bridge".format(robot["name"])]
        assert pubs["/livox/imu"] == pubs["/livox/lidar"]
        for key in ("navigation_map_yaml_path", "persist_state_dir", "robot_config_yaml_path",
                    "costmap_common_config_yaml_path", "rpp_config_yaml_path"):
            value = rospy.get_param("/grinder_scheduler/" + key)
            assert "/sim_fleet/" + robot["name"] + "/" in value, (key, value)
        assert rospy.get_param("/grinder_scheduler/sl_linka_control_port") == robot["control_port"]
        for port in (robot["app_port"], robot["control_port"]):
            with socket.create_connection(("127.0.0.1", port), timeout=3):
                pass
        if map_cycle:
            rospy.wait_for_service("/super_lio_mode/get_status", timeout=20)
            state = rospy.ServiceProxy("/super_lio_mode/get_status", GetSuperLioStatus)()
            assert state.state == "IDLE", "Map-cycle requires idle robots; will not stop an existing map"
            result = rospy.ServiceProxy("/super_lio_mode/start_mapping", Trigger)()
            started = bool(result.success)
            assert started, result.message
            grid = rospy.wait_for_message("/map", OccupancyGrid, timeout=20)
            assert grid.info.width > 0 and grid.info.height > 0
        results.put({"robot": robot["name"], "ok": True, "lidar_points": cloud.point_num,
                     "mapping": map_cycle})
    except Exception as exc:
        results.put({"robot": robot["name"], "ok": False, "error": repr(exc)})
    finally:
        if started:
            rospy.ServiceProxy("/super_lio_mode/stop", Trigger)()
        rospy.signal_shutdown("fleet check complete")


class AppPair:
    def __init__(self, robot, workspace):
        sys.path.insert(0, str(workspace.parent / "third_party/sl_linka/sdk/python"))
        from sl_link.frame.frame import SlFrame, SlFrameParser
        from sl_link.message_gen import sl_link_pb2 as pb
        self.pb, self.frame, self.seq = pb, SlFrame, 0
        self.sockets, self.parsers = [], []
        try:
            for port in (robot["app_port"], robot["control_port"]):
                self.sockets.append(socket.create_connection(("127.0.0.1", port), timeout=3))
                self.parsers.append(SlFrameParser())
            opened = self.call(0, pb.MSG_ID_CHANNEL_SESSION_REQUEST, pb.ChannelSessionRequest(
                action=pb.CHANNEL_SESSION_OPEN, role=pb.CHANNEL_ROLE_DATA), pb.ChannelSessionResponse)
            assert opened.result == pb.RESULT_SUCCESS, opened.message
            bound = self.call(1, pb.MSG_ID_CHANNEL_SESSION_REQUEST, pb.ChannelSessionRequest(
                action=pb.CHANNEL_SESSION_BIND, role=pb.CHANNEL_ROLE_CONTROL, token=opened.token), pb.ChannelSessionResponse)
            assert bound.result == pb.RESULT_SUCCESS, bound.message
        except Exception:
            self.close()
            raise

    def call(self, channel, message_id, request, response_type):
        self.seq = self.seq % 65535 + 1
        self.sockets[channel].sendall(self.frame(version=1, flags=0, seq=self.seq, ack_seq=0,
            src_id=self.pb.DEVICE_APP, dst_id=self.pb.DEVICE_LOWER, comp_id=self.pb.COMP_CONTROL,
            msg_id=message_id, payload=request.SerializeToString()).pack())
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            data = self.sockets[channel].recv(65536)
            if not data:
                break
            for frame in self.parsers[channel].parse(data):
                if frame.ack_seq == self.seq:
                    return response_type.FromString(frame.payload)
        raise RuntimeError("APP response timed out")

    def manual(self, moving):
        request = self.pb.ControlCommand()
        request.manual_drive.speed_ratio = 1 if moving else 0
        request.manual_drive.remote_y = -1 if moving else 0
        request.manual_drive.max_speed_mps = .08
        response = self.call(1, self.pb.MSG_ID_CONTROL_COMMAND, request, self.pb.ControlCommandResponse)
        assert response.result == self.pb.RESULT_SUCCESS, response.message

    def heartbeat(self):
        for index, role in enumerate((self.pb.CHANNEL_ROLE_DATA, self.pb.CHANNEL_ROLE_CONTROL)):
            response = self.call(index, self.pb.MSG_ID_CHANNEL_SESSION_REQUEST,
                self.pb.ChannelSessionRequest(action=self.pb.CHANNEL_SESSION_HEARTBEAT, role=role),
                self.pb.ChannelSessionResponse)
            assert response.result == self.pb.RESULT_SUCCESS, response.message

    def close(self):
        for peer in self.sockets:
            peer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", default=str(Path(__file__).resolve().parents[3] / "temp/sim_fleet/manifest.json"))
    parser.add_argument("--map-cycle", action="store_true")
    parser.add_argument("--drive", action="store_true")
    parser.add_argument("--watchdog", action="store_true", help="with --drive, pause world clock to verify wall-time stop")
    args = parser.parse_args()
    if args.watchdog and not args.drive:
        parser.error("--watchdog requires --drive")
    manifest = json.loads(Path(args.manifest).read_text())
    for port in [manifest["world_master_port"]] + [r["master_port"] for r in manifest["robots"]]:
        with socket.create_connection(("127.0.0.1", port), timeout=3):
            pass
    ctx = mp.get_context("spawn")
    results, processes = ctx.Queue(), []
    for robot in manifest["robots"]:
        process = ctx.Process(target=robot_check, args=(robot, args.map_cycle, results))
        processes.append(process)
        process.start()
    failed = False
    try:
        for _ in processes:
            result = results.get(timeout=110)
            print(json.dumps(result), flush=True)
            failed |= not result["ok"]
    finally:
        for process in processes:
            process.join(10)
            if process.is_alive():
                process.terminate()
                failed = True
    if failed:
        raise RuntimeError("fleet checks failed")
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:{}/".format(manifest["world_master_port"])
    import rospy
    from gazebo_msgs.srv import GetModelState, GetWorldProperties
    from geometry_msgs.msg import Twist
    from std_srvs.srv import Empty
    rospy.init_node("check_fleet_world", anonymous=True)
    models = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)().model_names
    assert all(robot["name"] in models for robot in manifest["robots"])
    print("Shared Gazebo models: " + ", ".join(models), flush=True)
    if args.drive:
        get_pose = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        def positions():
            answer = {}
            for robot in manifest["robots"]:
                response = get_pose(robot["name"], "world")
                assert response.success
                answer[robot["name"]] = (response.pose.position.x, response.pose.position.y)
            return answer
        for robot in manifest["robots"]:
            pair = AppPair(robot, Path(manifest["workspace"]))
            before = positions()
            paused = False
            try:
                start, last_heartbeat = time.monotonic(), 0
                while time.monotonic() - start < 3:
                    if time.monotonic() - last_heartbeat > .8:
                        pair.heartbeat()
                        last_heartbeat = time.monotonic()
                    pair.manual(True)
                    time.sleep(.1)
                if args.watchdog and robot == manifest["robots"][0]:
                    outputs = []
                    sub = rospy.Subscriber("/" + robot["name"] + "/grinder/sim/cmd_vel", Twist,
                        lambda message: outputs.append((time.monotonic(), message.linear.x, message.angular.z)))
                    time.sleep(.1)
                    pair.heartbeat()
                    rospy.ServiceProxy("/gazebo/pause_physics", Empty)()
                    paused = True
                    paused_at, sim_stamp = time.monotonic(), rospy.Time.now().to_sec()
                    # Keep the actual APP manual-receive lease alive. Only
                    # chassis output timers/clock stop, not the TCP session.
                    while time.monotonic() - paused_at < 1.0:
                        pair.manual(True)
                        time.sleep(.1)
                    zeros = [t for t, v, w in outputs if t >= paused_at and abs(v) + abs(w) < 1e-8]
                    assert zeros and zeros[0] - paused_at < .85, outputs[-5:]
                    assert abs(rospy.Time.now().to_sec() - sim_stamp) < .05, "clock did not pause"
                    sub.unregister()
                    print("Wall-time command watchdog stopped with paused clock in {:.3f}s".format(zeros[0] - paused_at), flush=True)
            finally:
                # The source command has expired and world output is already
                # zero. Resume timers before awaiting a protocol stop ACK.
                if paused:
                    rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
                    paused = False
                try:
                    pair.manual(False)
                finally:
                    pair.close()
            time.sleep(.5)
            after = positions()
            distances = {name: math.hypot(after[name][0] - point[0], after[name][1] - point[1])
                         for name, point in before.items()}
            assert distances[robot["name"]] > .02, distances
            assert all(value < .02 for name, value in distances.items() if name != robot["name"]), distances
            print("APP control isolation {}: {}".format(robot["name"], distances), flush=True)


if __name__ == "__main__":
    main()
