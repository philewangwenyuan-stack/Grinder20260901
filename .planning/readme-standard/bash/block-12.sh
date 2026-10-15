cd "$GRINDER_WS"
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=livox_ros_driver2 \
  -DCMAKE_BUILD_TYPE=Release -j2 -l2
source devel/setup.bash
