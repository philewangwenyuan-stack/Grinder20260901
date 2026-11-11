"""Simulation fleet configuration/URDF isolation, without a ROS dependency."""
import math
import re
import xml.etree.ElementTree as ET


def port(value):
    if isinstance(value, bool) or int(value) != float(value) or not 1024 <= int(value) <= 65535:
        raise ValueError("fleet ports must be integers in 1024..65535")
    return int(value)


def pose(value):
    if len(value) != 3 or not all(math.isfinite(float(v)) for v in value):
        raise ValueError("spawn must contain finite x, y, yaw")
    return [float(v) for v in value]


def validate_fleet(config, count, base_pose=(0, 0, 0)):
    if count not in (2, 3):
        raise ValueError("select 2 or 3 robots")
    result = dict(config)
    speed = float(result.get("sim_speed", 1))
    if not math.isfinite(speed) or not 0 < speed <= 10:
        raise ValueError("sim_speed must be in (0, 10]")
    result["sim_speed"] = speed
    if result.get("lidar_backend", "gpu") not in ("cpu", "gpu"):
        raise ValueError("lidar_backend must be cpu or gpu")
    for key, minimum, default in (("lidar_horizontal_samples", 4, 360),
                                  ("lidar_vertical_samples", 2, 20)):
        value = result.get(key, default)
        if isinstance(value, bool) or int(value) != float(value) or not minimum <= int(value) <= 4096:
            raise ValueError("invalid " + key)
        result[key] = int(value)
    distance = float(result.get("lidar_max_range", 45))
    if not math.isfinite(distance) or distance <= 1.5:
        raise ValueError("lidar_max_range must exceed the LIO blind distance")
    result["lidar_max_range"] = distance
    used_ports, names = set(), set()
    for key, default in (("world_master_port", 11411), ("gazebo_master_port", 11445)):
        result[key] = port(result.get(key, default))
        if result[key] in used_ports:
            raise ValueError("duplicate port")
        used_ports.add(result[key])
    robots = result.get("robots", [])[:count]
    if len(robots) != count:
        raise ValueError("fleet config does not define enough robots")
    base = pose(base_pose)
    result["robots"] = []
    for original in robots:
        robot = dict(original)
        name = robot.get("name", "")
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name) or name in names:
            raise ValueError("robot names must be unique ROS/path-safe names")
        names.add(name)
        for key in ("master_port", "app_port", "control_port"):
            robot[key] = port(robot[key])
            if robot[key] in used_ports:
                raise ValueError("duplicate port: " + str(robot[key]))
            used_ports.add(robot[key])
        if robot["control_port"] != robot["app_port"] + 1:
            raise ValueError("APP expects control_port = app_port + 1")
        offset = pose(robot.get("spawn_offset", [0, 0, 0]))
        robot["spawn"] = pose(robot.get("spawn", [a + b for a, b in zip(base, offset)]))
        for other in result["robots"]:
            if math.hypot(robot["spawn"][0] - other["spawn"][0],
                          robot["spawn"][1] - other["spawn"][1]) < 2.0:
                raise ValueError("spawn positions must be at least 2m apart")
        result["robots"].append(robot)
    return result


def isolate_urdf(text, name, cameras=False):
    """Patch a generated URDF only; preserve physical and sensor geometry.

    TF frame names deliberately remain local: each robot has a separate master.
    Only the shared Gazebo process uses namespaced topic names.
    """
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name):
        raise ValueError("invalid robot name")
    root = ET.fromstring(text)
    root.set("name", name)
    for gazebo in root.findall("gazebo"):
        for sensor in list(gazebo.findall("sensor")):
            if sensor.get("type") == "camera" and not cameras:
                gazebo.remove(sensor)
    for plugin in root.iter("plugin"):
        filename = plugin.get("filename", "")
        namespace = plugin.find("robotNamespace")
        if namespace is not None:
            namespace.text = "/" + name
        # Camera plugins without robotNamespace otherwise also share camera_info.
        if "camera" in filename and namespace is None:
            ET.SubElement(plugin, "robotNamespace").text = "/" + name
        for key in ("commandTopic", "odometryTopic", "topicName", "imageTopicName", "cameraInfoTopicName"):
            node = plugin.find(key)
            if node is not None and node.text:
                node.text = "/" + name + "/" + node.text.lstrip("/")
        if filename == "libgrinder_mid360_sensor.so":
            node = plugin.find("topic_name")
            if node is None:
                node = ET.SubElement(plugin, "topic_name")
            node.text = "/" + name + "/livox/lidar"
    return ET.tostring(root, encoding="unicode")


def sensor_routes(name, cameras=False):
    """(shared-world source, robot destination, message type, queue capacity)."""
    topics = [("/livox/lidar", "livox_ros_driver2/CustomMsg", 3),
              ("/livox/imu", "sensor_msgs/Imu", 400),
              ("/grinder/sim/ground_truth/odom", "nav_msgs/Odometry", 30),
              ("/joint_states", "sensor_msgs/JointState", 30)]
    if cameras:
        for side in ("left", "right"):
            topics.append(("/grinder/sim/" + side + "/image_raw", "sensor_msgs/Image", 2))
    return [("/clock", "/clock", "rosgraph_msgs/Clock", 30)] + [
        ("/" + name + topic, topic, message_type, capacity)
        for topic, message_type, capacity in topics]


class CommandDeadline:
    """Wall-time watchdog; never depends on a paused simulation clock."""
    def __init__(self, timeout=0.5, max_age=0.25):
        self.timeout, self.max_age = timeout, max_age
        self.last_received = None

    def accept(self, sent, now):
        if not math.isfinite(sent) or not 0 <= now - sent <= self.max_age:
            return False
        self.last_received = sent
        return True

    def expired(self, now):
        return self.last_received is None or now - self.last_received > self.timeout
