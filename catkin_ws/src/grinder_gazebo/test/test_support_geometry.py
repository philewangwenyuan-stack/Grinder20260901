"""Offline regression for caster ride height; run with ROS Noetic/xacro sourced.

No ROS master or Gazebo instance is started, and no motion commands are sent.
"""
import math
from pathlib import Path
import shutil
import subprocess
import unittest
import xml.etree.ElementTree as ET


def xyz(node):
    return [float(x) for x in node.get("xyz", "0 0 0").split()] if node is not None else [0.0] * 3


class SupportGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        command = shutil.which("xacro")
        if not command:
            raise unittest.SkipTest("Source ROS Noetic with xacro before running this test")
        source = Path(__file__).resolve().parents[1] / "urdf/x920_grinder.urdf.xacro"
        cls.robot = ET.fromstring(subprocess.check_output([command, str(source)]))
        cls.links = {link.get("name"): link for link in cls.robot.findall("link")}
        cls.joints = {joint.get("name"): joint for joint in cls.robot.findall("joint")}

    def test_front_support_has_constant_ground_clearance_through_full_rotation(self):
        link = self.links["grinding_disc_caster_wheel_link"]
        joint = self.joints["grinding_disc_caster_wheel_joint"]
        radius = float(link.find("collision/geometry/sphere").get("radius"))
        wheel_radius = float(self.links["left_wheel_link"].find("collision/geometry/cylinder").get("radius"))
        mount = xyz(joint.find("origin"))
        center = xyz(link.find("collision/origin"))
        clearances = []
        for step in range(73):
            angle = step * math.pi / 36
            # The generated rolling axis is +Y; rotate the actual collision offset.
            z = -math.sin(angle) * center[0] + math.cos(angle) * center[2]
            clearances.append(wheel_radius + mount[2] + z - radius)
        self.assertLess(max(clearances) - min(clearances), 1e-9,
                        "Caster rotation changes support height and pitches the chassis")
        self.assertLess(max(abs(value) for value in clearances), 1e-9,
                        "Caster and drive-wheel bottoms must share the nominal ground plane")

    def test_mass_and_center_of_gravity_remain_at_the_declared_vehicle_target(self):
        # All massive child-link mounting frames have identity rotation in this model.
        poses = {"base_link": [0.0, 0.0, 0.0]}
        pending = list(self.robot.findall("joint"))
        while pending:
            progressed = False
            for joint in pending[:]:
                parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
                if parent not in poses:
                    continue
                poses[child] = [a + b for a, b in zip(poses[parent], xyz(joint.find("origin")))]
                pending.remove(joint)
                progressed = True
            self.assertTrue(progressed, "Model joint tree contains an unresolved parent")
        total = 0.0
        weighted = [0.0] * 3
        for name, link in self.links.items():
            inertia = link.find("inertial")
            if inertia is None:
                continue
            mass = float(inertia.find("mass").get("value"))
            center = [a + b for a, b in zip(poses[name], xyz(inertia.find("origin")))]
            total += mass
            weighted = [a + mass * b for a, b in zip(weighted, center)]
        wheel_radius = float(self.links["left_wheel_link"].find("collision/geometry/cylinder").get("radius"))
        self.assertAlmostEqual(total, 800.0)
        self.assertAlmostEqual(weighted[0] / total, 0.55)
        self.assertAlmostEqual(weighted[1] / total, 0.0)
        self.assertAlmostEqual(weighted[2] / total + wheel_radius, 0.04)


if __name__ == "__main__":
    unittest.main()
