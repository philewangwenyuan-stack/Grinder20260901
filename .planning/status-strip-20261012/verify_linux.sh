#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
cd /mnt/e/CODE/C++/Grinder
indicator_ws=$(mktemp -d /tmp/grinder-indicator-catkin-XXXXXX)
printf '%s\n' "$indicator_ws" > /tmp/grinder-indicator-catkin-path
mkdir "$indicator_ws/src"
ln -s /mnt/e/CODE/C++/Grinder/catkin_ws/src/CMakeLists.txt "$indicator_ws/src/CMakeLists.txt"
ln -s /mnt/e/CODE/C++/Grinder/catkin_ws/src/grinder_chassis_driver "$indicator_ws/src/grinder_chassis_driver"
ln -s /mnt/e/CODE/C++/Grinder/catkin_ws/src/grinder_scheduler "$indicator_ws/src/grinder_scheduler"
cd "$indicator_ws"
catkin_make -DCATKIN_WHITELIST_PACKAGES=grinder_chassis_driver -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2 -l2
source devel/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=grinder_scheduler -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2 -l2
