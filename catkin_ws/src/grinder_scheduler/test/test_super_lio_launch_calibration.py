"""Offline launch-chain regression: measured mounting drives TF and LIO alike."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

SRC = Path(__file__).resolve().parents[2]
PACKAGES = {
    'grinder_scheduler': SRC / 'grinder_scheduler',
    'cloud_to_occupancy_grid': SRC / 'mapping/cloud_to_occupancy_grid',
    'super_lio': SRC / 'localization/Super-LIO/src/super_lio',
}
AXES = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


def expand(value, arguments):
    for key, replacement in arguments.items():
        value = value.replace('$(arg ' + key + ')', replacement)
    for key, path in PACKAGES.items():
        value = value.replace('$(find ' + key + ')', str(path))
    return value


def collect(path, supplied):
    root = ET.parse(path).getroot()
    arguments = dict(supplied)
    for arg in root.findall('arg'):
        arguments.setdefault(arg.attrib['name'], arg.attrib.get('default', ''))
    params, mounting_nodes = {}, []
    for param in root.findall('param'):
        params[param.attrib['name']] = expand(param.attrib.get('value', ''), arguments)
    for node in root.findall('node'):
        if node.attrib.get('name') == 'base_to_laser':
            mounting_nodes.append(expand(node.attrib['args'], arguments).split())
    for include in root.findall('include'):
        filename = expand(include.attrib['file'], arguments)
        # Only inspect the LIO mounting chain, not loop/map-server startup.
        if Path(filename).name not in ('mid360_mapping.launch', 'Livox_mid360.launch', 'relocation.launch'):
            continue
        children = {arg.attrib['name']: expand(arg.attrib['value'], arguments)
                    for arg in include.findall('arg')}
        more_params, more_nodes = collect(filename, children)
        params.update(more_params)
        mounting_nodes.extend(more_nodes)
    return params, mounting_nodes


class SuperLioLaunchCalibrationTest(unittest.TestCase):
    def test_all_entrypoints_share_one_calibration_and_one_static_tf(self):
        mounting = dict(zip(AXES, ('0.1', '-0.02', '0.32', '0.04', '-0.07', '0.35')))
        supplied = {'base_to_laser_' + key: value for key, value in mounting.items()}
        for package, filename in (
            ('grinder_scheduler', 'super_lio_mapping_managed.launch'),
            ('grinder_scheduler', 'super_lio_localization_managed.launch'),
            ('cloud_to_occupancy_grid', 'mid360_mapping.launch'),
            ('super_lio', 'Livox_mid360.launch'),
            ('super_lio', 'relocation.launch'),
        ):
            with self.subTest(filename=filename):
                params, nodes = collect(PACKAGES[package] / 'launch' / filename, supplied)
                self.assertEqual(len(nodes), 1)
                self.assertEqual(nodes[0][-2:], ['base_link', 'base_laser_link'])
                expected = [mounting[key] for key in ('x', 'y', 'z', 'yaw', 'pitch', 'roll')]
                self.assertEqual(nodes[0][:6], expected)
                for key in AXES:
                    self.assertEqual(params['/lio/extrinsic/base_to_laser_' + key], mounting[key])


if __name__ == '__main__':
    unittest.main()
