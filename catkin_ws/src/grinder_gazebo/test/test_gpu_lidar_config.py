"""Xacro regression for CPU fallback and GPU guard texels. Requires ROS xacro."""
import math
from pathlib import Path
import shutil
import subprocess
import unittest
import xml.etree.ElementTree as ET


@unittest.skipUnless(shutil.which('xacro'), 'ROS xacro is required')
class GpuLidarConfigTests(unittest.TestCase):
    def sensor(self, backend, offset=1):
        path = Path(__file__).resolve().parents[1] / 'urdf/x920_grinder.urdf.xacro'
        xml = subprocess.check_output(['xacro', str(path), 'lidar_backend:=' + backend,
                                       'gpu_ray_sample_offset:=' + str(offset)])
        root = ET.fromstring(xml)
        return root.find(".//sensor[@name='mid360s_lidar']")

    def test_cpu_keeps_original_grid(self):
        sensor = self.sensor('cpu')
        self.assertEqual(sensor.attrib['type'], 'ray')
        self.assertEqual(int(sensor.findtext('plugin/sample_offset')), 0)
        self.check_grid(sensor, 0)

    def test_gpu_preserves_requested_grid_with_guard_texels(self):
        sensor = self.sensor('gpu')
        self.assertEqual(sensor.attrib['type'], 'gpu_ray')
        self.assertEqual(int(sensor.findtext('plugin/sample_offset')), 1)
        self.check_grid(sensor, 1)

    def test_gpu_zero_offset_supports_other_render_backends(self):
        self.check_grid(self.sensor('gpu', 0), 0)

    def check_grid(self, sensor, offset):
        for axis, count, low, high in [('horizontal', 360, -math.pi, math.pi-2*math.pi/360),
                                        ('vertical', 20, -7*math.pi/180, 52*math.pi/180)]:
            scan = sensor.find('ray/scan/' + axis)
            n = int(scan.findtext('samples'))
            lo = float(scan.findtext('min_angle'))
            hi = float(scan.findtext('max_angle'))
            self.assertEqual(n-offset, count)
            self.assertAlmostEqual(lo + offset*(hi-lo)/(n-1), low)
            self.assertAlmostEqual(hi, high)
        self.assertEqual(sensor.findtext('ray/noise/type'), 'gaussian')
        self.assertAlmostEqual(float(sensor.findtext('ray/noise/stddev')), .01)
        self.assertEqual(float(sensor.findtext('update_rate')), 10)


if __name__ == '__main__':
    unittest.main()
