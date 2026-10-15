from pathlib import Path

root = Path(__file__).resolve().parents[2]
axes = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')

def edit(relative, old, new):
    path = root / relative
    text = path.read_text(encoding='utf-8')
    assert old in text, (relative, old)
    path.write_text(text.replace(old, new, 1), encoding='utf-8')

lio = 'catkin_ws/src/localization/Super-LIO/src/super_lio/'
for name, config in [('Livox_mid360.launch', 'livox_360.yaml'), ('relocation.launch', 'relocation.yaml')]:
    old = f'  <rosparam file="$(find super_lio)/config/{config}" command="load"/>\n'
    params = ''.join(f'  <param name="/lio/extrinsic/base_to_laser_{axis}" value="$(arg base_to_laser_{axis})" type="double" />\n' for axis in axes)
    edit(lio + 'launch/' + name, old, old + params)

arg_defs = ''.join(f'  <arg name="base_to_laser_{axis}" default="0.0" />\n' for axis in axes)
arg_pass = ''.join(f'    <arg name="base_to_laser_{axis}" value="$(arg base_to_laser_{axis})" />\n' for axis in axes)
for relative, anchor, include in [
    ('catkin_ws/src/grinder_scheduler/launch/super_lio_mapping_managed.launch', '  <arg name="staging_dir" />\n', '    <arg name="rviz" value="false" />\n'),
    ('catkin_ws/src/mapping/cloud_to_occupancy_grid/launch/mid360_mapping.launch', '  <arg name="rviz" default="true"/>\n', '    <arg name="rviz" value="false"/>\n')]:
    edit(relative, anchor, anchor + arg_defs)
    edit(relative, include, include + arg_pass)

manager = 'catkin_ws/src/grinder_scheduler/src/grinder_scheduler/super_lio_mode_manager.py'
helper = '''    @staticmethod
    def _base_laser_launch_args():
        # One calibration source for fixed TF and LIO output in both modes.
        return ["{}:={}".format(key, rospy.get_param("~" + key, 0.0))
                for key in ("base_to_laser_x", "base_to_laser_y", "base_to_laser_z",
                            "base_to_laser_roll", "base_to_laser_pitch", "base_to_laser_yaw")]

'''
edit(manager, '    def _start_mapping_locked(self):\n', helper + '    def _start_mapping_locked(self):\n')
edit(manager, '            "autosave:={}".format(str(self._autosave).lower()),\n        ]\n', '            "autosave:={}".format(str(self._autosave).lower()),\n        ]\n        args.extend(self._base_laser_launch_args())\n')
old = '''        for key in (
            "base_to_laser_x", "base_to_laser_y", "base_to_laser_z",
            "base_to_laser_roll", "base_to_laser_pitch", "base_to_laser_yaw",
        ):
            args.append("{}:={}".format(key, rospy.get_param("~" + key, 0.0)))
'''
edit(manager, old, '        args.extend(self._base_laser_launch_args())\n')

for config in ['livox_360.yaml', 'relocation.yaml']:
    defaults = ''.join(f'    base_to_laser_{axis}: 0.0\n' for axis in axes)
    edit(lio + 'config/' + config, '  extrinsic:\n', '  extrinsic:\n    # Same measured T_B_L as the launch static TF; metres and radians.\n' + defaults)
