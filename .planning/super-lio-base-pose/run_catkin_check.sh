#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
set -u
root=/mnt/e/CODE/C++/Grinder
check_ws=$(mktemp -d /tmp/grinder-base-pose-catkin.XXXXXX)
mkdir -p "$check_ws/src"
ln -s "$root/catkin_ws/src/localization/Super-LIO/src/basic" "$check_ws/src/basic"
ln -s "$root/catkin_ws/src/localization/Super-LIO/src/super_lio" "$check_ws/src/super_lio"
ln -s "$root/catkin_ws/src/localization/super_lio_loop" "$check_ws/src/super_lio_loop"
cd "$check_ws"
echo "Validation workspace: $check_ws"
catkin_make -j2 -DCATKIN_ENABLE_TESTING=ON -DPYTHON_EXECUTABLE=/usr/bin/python3
