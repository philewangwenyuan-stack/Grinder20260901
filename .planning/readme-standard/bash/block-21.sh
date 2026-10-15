roslaunch cloud_to_occupancy_grid mid360_mapping.launch \
  rviz:=true save_3d_map:=true loop_closure:=true loop_backend:=industrial \
  base_to_laser_x:="$BASE_TO_LASER_X" base_to_laser_y:="$BASE_TO_LASER_Y" \
  base_to_laser_z:="$BASE_TO_LASER_Z" base_to_laser_roll:="$BASE_TO_LASER_ROLL" \
  base_to_laser_pitch:="$BASE_TO_LASER_PITCH" base_to_laser_yaw:="$BASE_TO_LASER_YAW"
