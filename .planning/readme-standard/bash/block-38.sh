cd "$GRINDER_BASE_DIR"
PYTHONPATH=catkin_ws/src/grinder_chassis_driver/src python3 -m unittest discover \
  -s catkin_ws/src/grinder_chassis_driver/test -p 'test_*.py'
PYTHONPATH=catkin_ws/src/grinder_scheduler/src python3 -m unittest discover \
  -s catkin_ws/src/grinder_scheduler/test -p 'test_super_lio*.py'
python3 catkin_ws/scripts/test_grinder_service.py -v
python3 -m compileall -q catkin_ws/src/grinder_scheduler/src/grinder_scheduler \
  catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver
