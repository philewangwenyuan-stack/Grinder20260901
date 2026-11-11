"""Explicit, bounded topic bridge between one world and one robot ROS master.

    A process imports/initializes rospy for exactly one master. No TF, map,
    task, chassis safety, or localization data crosses between robot masters.
"""
import argparse
import io
import multiprocessing as mp
import os
import queue
import signal
import time

from .fleet_config import CommandDeadline, sensor_routes


def put_latest(channel, item):
    try:
        channel.put_nowait(item)
    except queue.Full:
        try:
            channel.get_nowait()
        except queue.Empty:
            pass
        try:
            channel.put_nowait(item)
        except queue.Full:
            pass


def serialize(message):
    if hasattr(message, "_buff"):
        return bytes(message._buff)
    data = io.BytesIO()
    message.serialize(data)
    return data.getvalue()


def endpoint(world, uri, name, cameras, channels, commands, stop, ready):
    os.environ["ROS_MASTER_URI"] = uri
    # Import only after setting this process's master.
    import rospy
    from roslib.message import get_message_class
    from geometry_msgs.msg import Twist
    rospy.init_node("fleet_{}_{}_bridge".format(name, "world" if world else "robot"),
                    disable_signals=True)
    routes = sensor_routes(name, cameras)
    subscribers, publishers = [], []
    command_pub = None
    try:
        for index, (source, destination, type_name, capacity) in enumerate(routes):
            message_class = get_message_class(type_name)
            if message_class is None:
                raise RuntimeError("missing ROS message: " + type_name)
            if world:
                def receive(message, channel=channels[index], expected=type_name):
                    if message._connection_header.get("type") != expected:
                        rospy.logerr("Fleet sensor type mismatch: %s", expected)
                        stop.set()
                        return
                    # rospy itself subscribes to /clock with a typed Clock in
                    # this process, so that callback receives Clock, not AnyMsg.
                    put_latest(channel, serialize(message))
                # Forward serialized sensor bytes. Deserializing every Livox
                # point into Python objects twice needlessly consumes a CPU core.
                subscribers.append(rospy.Subscriber(source, rospy.AnyMsg, receive,
                                                    queue_size=capacity, tcp_nodelay=True))
            else:
                publishers.append((rospy.Publisher(destination, message_class,
                                                   queue_size=capacity), message_class))
        if world:
            command_pub = rospy.Publisher("/" + name + "/grinder/sim/cmd_vel", Twist, queue_size=1)
        else:
            def receive_command(message):
                put_latest(commands, (time.monotonic(), serialize(message)))
            subscribers.append(rospy.Subscriber("/grinder/sim/cmd_vel", Twist,
                                                receive_command, queue_size=1, tcp_nodelay=True))
        deadline, last_zero = CommandDeadline(), 0.0
        ready.set()
        while not stop.is_set() and not rospy.is_shutdown():
            if world:
                try:
                    sent, data = commands.get(timeout=0.01)
                    if deadline.accept(sent, time.monotonic()):
                        command_pub.publish(Twist().deserialize(data))
                except queue.Empty:
                    pass
                now = time.monotonic()
                if deadline.expired(now) and now - last_zero >= 0.05:
                    command_pub.publish(Twist())
                    last_zero = now
            else:
                received = False
                # Independent queues prevent a large lidar frame or camera
                # backlog from starving IMU and clock delivery.
                for channel, (publisher, message_class) in zip(channels, publishers):
                    for _ in range(40):
                        try:
                            data = channel.get_nowait()
                        except queue.Empty:
                            break
                        # Typed publisher supplies the correct ROS handshake;
                        # AnyMsg preserves original bytes, stamps and frame IDs.
                        publisher.publish(rospy.AnyMsg().deserialize(data))
                        received = True
                if not received:
                    stop.wait(0.002)
    finally:
        if command_pub is not None:
            for _ in range(3):
                command_pub.publish(Twist())
                time.sleep(0.02)
        rospy.signal_shutdown("fleet bridge stopped")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world-uri", required=True)
    parser.add_argument("--robot-uri", required=True)
    parser.add_argument("--name", required=True)
    parser.add_argument("--cameras", action="store_true")
    args = parser.parse_args()
    ctx = mp.get_context("spawn")
    channels = [ctx.Queue(maxsize=capacity) for _, _, _, capacity in sensor_routes(args.name, args.cameras)]
    commands, stop = ctx.Queue(maxsize=2), ctx.Event()
    processes, readiness = [], []
    for world, uri in ((True, args.world_uri), (False, args.robot_uri)):
        ready = ctx.Event()
        process = ctx.Process(target=endpoint, args=(world, uri, args.name, args.cameras,
                                                    channels, commands, stop, ready), daemon=True)
        processes.append(process)
        readiness.append(ready)
        process.start()
    def shutdown(_signum, _frame):
        stop.set()
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    failed = False
    try:
        started = time.monotonic()
        while not stop.wait(0.1):
            if any(not process.is_alive() for process in processes):
                failed = True
                break
            if time.monotonic() - started > 30 and not all(event.is_set() for event in readiness):
                failed = True
                break
    finally:
        stop.set()
        for process in processes:
            process.join(3)
            if process.is_alive():
                process.terminate()
                process.join(2)
        for channel in channels + [commands]:
            channel.cancel_join_thread()
            channel.close()
    return 1 if failed else 0
