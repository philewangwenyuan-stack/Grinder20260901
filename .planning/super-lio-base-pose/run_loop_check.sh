#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
check_ws=/tmp/grinder-base-pose-catkin.tAU4Xh
cd "$check_ws"
catkin_make -j1 -DCATKIN_WHITELIST_PACKAGES=super_lio_loop \
  -DCATKIN_ENABLE_TESTING=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DSUPER_LIO_REQUIRE_INDUSTRIAL_BACKEND=OFF > /tmp/grinder-base-loop-build.log 2>&1 || {
    tail -45 /tmp/grinder-base-loop-build.log
    exit 1
  }
tail -25 /tmp/grinder-base-loop-build.log
