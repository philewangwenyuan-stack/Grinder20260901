"""Offline regression for scan-only dust filtering through every launch chain."""
import ast
import unittest

import yaml

from test_super_lio_launch_calibration import collect, PACKAGES


ENTRYPOINTS = (
    ('grinder_scheduler', 'super_lio_mapping_managed.launch'),
    ('grinder_scheduler', 'super_lio_localization_managed.launch'),
    ('cloud_to_occupancy_grid', 'mid360_mapping.launch'),
    ('super_lio', 'Livox_mid360.launch'),
    ('super_lio', 'relocation.launch'),
)


class ScanFilterConfigurationTest(unittest.TestCase):
    def test_default_and_explicit_values_reach_algorithm(self):
        for package, name in ENTRYPOINTS:
            for enabled in (None, 'false', 'true'):
                with self.subTest(entrypoint=name, enabled=enabled):
                    supplied = {} if enabled is None else {'scan_filter_livox_dust': enabled}
                    params, _ = collect(PACKAGES[package] / 'launch' / name, supplied)
                    self.assertEqual(enabled or 'true', params['/lio/scan/filter_livox_dust'])

    def test_yaml_defaults_are_enabled_booleans(self):
        scheduler = PACKAGES['grinder_scheduler'] / 'config/super_lio.yaml'
        self.assertIs(yaml.safe_load(scheduler.read_text(encoding='utf-8'))[
            'super_lio_scan_filter_livox_dust'], True)
        for name in ('livox_360.yaml', 'relocation.yaml'):
            path = PACKAGES['super_lio'] / 'config' / name
            self.assertIs(yaml.safe_load(path.read_text(encoding='utf-8'))[
                'lio']['scan']['filter_livox_dust'], True)

    def test_manager_default_and_override(self):
        path = PACKAGES['grinder_scheduler'] / 'src/grinder_scheduler/super_lio_mode_manager.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        cls = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                   and node.name == 'SuperLioModeManager')
        method = next(node for node in cls.body if isinstance(node, ast.FunctionDef)
                      and node.name == '_scan_filter_launch_args')
        method.decorator_list = []
        values = {}

        class Ros:
            @staticmethod
            def get_param(name, default):
                return values.get(name, default)

        namespace = {'rospy': Ros}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
        arguments = namespace['_scan_filter_launch_args']
        self.assertEqual(['scan_filter_livox_dust:=true'], arguments())
        values['~super_lio_scan_filter_livox_dust'] = False
        self.assertEqual(['scan_filter_livox_dust:=false'], arguments())
        values['~super_lio_scan_filter_livox_dust'] = True
        self.assertEqual(['scan_filter_livox_dust:=true'], arguments())


if __name__ == '__main__':
    unittest.main()
