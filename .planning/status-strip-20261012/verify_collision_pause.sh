#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
cd /mnt/e/CODE/C++/Grinder
g++ -std=c++11 -Wall -Wextra -Werror -Ifirmware/status_strip/include \
  firmware/status_strip/test/protocol_host.cpp -o /tmp/grinder_indicator_protocol_host
INDICATOR_PROTOCOL_TEST_EXE=/tmp/grinder_indicator_protocol_host python3 -m unittest discover \
  -s catkin_ws/src/grinder_scheduler/test -p 'test_status_indicator*.py'
PYTHONPATH=catkin_ws/src/grinder_chassis_driver/src python3 -m unittest discover \
  -s catkin_ws/src/grinder_chassis_driver/test -p 'test_*.py'
