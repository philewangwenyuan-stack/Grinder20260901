rosparam get /lio/ros/legacy_tf
rosparam get /lio/extrinsic
rostopic echo -n 1 /lio/odom
rosrun tf tf_echo map base_link
rosrun tf tf_echo base_link base_laser_link
