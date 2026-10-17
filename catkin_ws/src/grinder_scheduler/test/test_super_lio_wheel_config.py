"""Offline validation of wheel arguments through all managed/direct launches."""
import ast
from pathlib import Path
import unittest
from test_super_lio_launch_calibration import collect, PACKAGES


class WheelLaunchTest(unittest.TestCase):
    def test_arguments_reach_algorithm_in_all_modes(self):
        for package, name in (('grinder_scheduler','super_lio_mapping_managed.launch'),
                              ('grinder_scheduler','super_lio_localization_managed.launch'),
                              ('cloud_to_occupancy_grid','mid360_mapping.launch'),
                              ('super_lio','Livox_mid360.launch'),('super_lio','relocation.launch')):
            for linear,angular in (('true','false'),('false','true'),('true','true')):
                args=dict(wheel_fusion_enabled='true',wheel_fuse_linear_velocity=linear,
                          wheel_fuse_angular_velocity=angular,wheel_twist_topic='/test/wheel',
                          wheel_max_age_sec='0.2',wheel_single_nis_gate='8',wheel_joint_nis_gate='11')
                params,_=collect(PACKAGES[package]/'launch'/name,args)
                self.assertEqual('true',params['/lio/wheel/enabled'])
                self.assertEqual(linear,params['/lio/wheel/fuse_linear_velocity'])
                self.assertEqual(angular,params['/lio/wheel/fuse_angular_velocity'])
                self.assertEqual('/test/wheel',params['/lio/wheel/twist_topic'])
                self.assertEqual('0.2',params['/lio/wheel/max_age_sec'])

    def test_defaults_disabled(self):
        for package,name in (('grinder_scheduler','super_lio_mapping_managed.launch'),
                             ('grinder_scheduler','super_lio_localization_managed.launch')):
            params,_=collect(PACKAGES[package]/'launch'/name,{})
            self.assertEqual('false',params['/lio/wheel/enabled'])
            self.assertEqual('true',params['/lio/wheel/fuse_linear_velocity'])
            self.assertEqual('false',params['/lio/wheel/fuse_angular_velocity'])
            self.assertEqual('/chassis/wheel_twist',params['/lio/wheel/twist_topic'])

    def test_manager_mode_selection_and_empty_configuration_rejected(self):
        p=PACKAGES['grinder_scheduler']/'src/grinder_scheduler/super_lio_mode_manager.py'
        tree=ast.parse(p.read_text(encoding='utf-8'))
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SuperLioModeManager')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_wheel_fusion_launch_args')
        method.decorator_list=[]
        values={'~super_lio_wheel_fusion_mapping_enabled':True}
        class Ros:
            @staticmethod
            def get_param(key,default):return values.get(key,default)
        ns={'rospy':Ros}
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(p),'exec'),ns)
        f=ns['_wheel_fusion_launch_args']
        self.assertIn('wheel_fusion_enabled:=true',f('mapping'))
        self.assertIn('wheel_fusion_enabled:=false',f('localization'))
        values['~super_lio_wheel_fuse_linear_velocity']=False
        with self.assertRaises(ValueError):f('mapping')


if __name__=='__main__':unittest.main()
