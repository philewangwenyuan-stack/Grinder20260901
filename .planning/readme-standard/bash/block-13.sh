cd "$GRINDER_WS"
CATKIN_JOBS=2 CATKIN_LOAD=2 PROFILE=mapping bash ./build_grinder_platform.sh
CATKIN_JOBS=2 CATKIN_LOAD=2 PROFILE=full bash ./build_grinder_platform.sh
source devel/setup.bash
