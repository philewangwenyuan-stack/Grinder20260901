MAP_DIR="$GRINDER_BASE_DIR/maps/实际地图ID"
roslaunch super_lio relocation.launch rviz:=true map_dir:="$MAP_DIR" map_name:=loc_map.pcd \
  base_to_laser_x:="$BASE_TO_LASER_X" base_to_laser_y:="$BASE_TO_LASER_Y" \
  base_to_laser_z:="$BASE_TO_LASER_Z" base_to_laser_roll:="$BASE_TO_LASER_ROLL" \
  base_to_laser_pitch:="$BASE_TO_LASER_PITCH" base_to_laser_yaw:="$BASE_TO_LASER_YAW"
